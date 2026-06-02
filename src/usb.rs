// USB XInput controller driver and task module using embassy-usb

use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::channel::Channel;
use embassy_sync::signal::Signal;
use embassy_usb::driver::{Endpoint, EndpointIn, EndpointOut};
use embassy_usb::{Builder, Config, UsbDevice};
use esp_hal::otg_fs::asynch::Driver as EspUsbDriver;

#[derive(Clone, Copy)]
pub struct UsbInFrame {
    pub report: [u8; 64],
    pub ble_instant: embassy_time::Instant,
}

/// Inbound controller reports destined for USB IN endpoint.
/// Keep only the latest report so stale frames do not build backlog behind USB poll timing.
pub static USB_REPORTS: Channel<CriticalSectionRawMutex, UsbInFrame, 1> = Channel::new();
/// Signal from the button task to trigger USB re-enumeration with the new mode.
/// BLE is NOT affected — only the USB peripheral is recycled.
pub static USB_REINIT: Signal<CriticalSectionRawMutex, ()> = Signal::new();

pub async fn queue_usb_report(report: [u8; 64], ble_instant: embassy_time::Instant) {
    let frame = UsbInFrame { report, ble_instant };
    if USB_REPORTS.try_send(frame).is_err() {
        let _ = USB_REPORTS.receive().await;
        let _ = USB_REPORTS.try_send(frame);
    }
}

#[derive(Clone, Copy)]
pub struct UsbOutFrame {
    pub mode: crate::storage::OutputMode,
    pub len: u8,
    pub data: [u8; 64],
}

/// Raw OUT reports received from USB host.
/// Consumed by a higher-level haptics bridge task.
pub static USB_OUT_FRAMES: Channel<CriticalSectionRawMutex, UsbOutFrame, 16> = Channel::new();

fn neutral_report(mode: crate::storage::OutputMode) -> [u8; 64] {
    match mode {
        crate::storage::OutputMode::XInput => {
            let mut r = [0u8; 64];
            r[1] = 0x14;
            r
        }
        crate::storage::OutputMode::SwitchPro => {
            let mut r = [0u8; 64];
            r[2] = 0x08;
            r[3] = 128;
            r[4] = 128;
            r[5] = 128;
            r[6] = 128;
            r
        }
        crate::storage::OutputMode::DualSense => {
            let mut r = [0u8; 64];
            r[0] = 0x01;
            r[1] = 128;
            r[2] = 128;
            r[3] = 128;
            r[4] = 128;
            r[7] = 0x08;
            r
        }
    }
}

/// Runs USB device management and report forwarding concurrently.
/// On USB_REINIT signal the USB hardware is torn down and rebuilt with whatever
/// mode is saved in flash — no full chip reset, so BLE stays connected.
#[embassy_executor::task]
#[allow(static_mut_refs)] // Safety: single-task exclusive access; previous UsbDevice dropped before reborrow
pub async fn usb_manager_task() {
    // Static buffers reused on every reinit.
    // Safety: this task is the only user; previous UsbDevice is dropped before reborrow.
    static mut EP_OUT_BUF: [u8; 1024] = [0u8; 1024];
    static mut CONFIG_BUF: [u8; 1024] = [0u8; 1024];
    static mut BOS_BUF:    [u8; 1024] = [0u8; 1024];
    static mut MSOS_BUF:   [u8; 1024] = [0u8; 1024];
    static mut CTRL_BUF:   [u8; 1024] = [0u8; 1024];

    loop {
        let mode = crate::storage::load_output_mode();
        log::info!("USB manager: starting USB for mode {:?}", mode);

        let (mut usb_device, mut write_ep, mut read_ep) = unsafe {
            // Steal peripheral tokens — safe because previous UsbDevice (if any) was dropped
            // at the end of the last iteration and we are the sole user of these peripherals.
            let usb0   = esp_hal::peripherals::USB0::steal();
            let gpio20 = esp_hal::peripherals::GPIO20::steal();
            let gpio19 = esp_hal::peripherals::GPIO19::steal();
            let usb = esp_hal::otg_fs::Usb::new(usb0, gpio20, gpio19);

            EP_OUT_BUF.fill(0);
            CONFIG_BUF.fill(0); BOS_BUF.fill(0); MSOS_BUF.fill(0); CTRL_BUF.fill(0);

            let driver = EspUsbDriver::new(
                usb,
                &mut EP_OUT_BUF,
                esp_hal::otg_fs::asynch::Config::default(),
            );

            setup_usb(driver, &mut CONFIG_BUF, &mut BOS_BUF, &mut MSOS_BUF, &mut CTRL_BUF, mode)
        };

        // Give the host an immediate, valid state after enumeration.
        queue_usb_report(neutral_report(mode), embassy_time::Instant::now()).await;

        let report_len: usize = match mode {
            crate::storage::OutputMode::XInput    => 20,
            crate::storage::OutputMode::SwitchPro => 8,
            crate::storage::OutputMode::DualSense => 64,
        };

        // Run the USB device FSM and the report-forwarding loop concurrently.
        // When USB_REINIT fires, both are cancelled and we loop to rebuild with the new mode.
        let device_fut = usb_device.run();
        let reporter_fut = async {
            let mut write_disabled_logged = false;
            loop {
                let frame = USB_REPORTS.receive().await;
                match write_ep.write(&frame.report[..report_len]).await {
                    Ok(_) => {
                        if write_disabled_logged {
                            log::info!("USB IN endpoint re-enabled; resuming report writes");
                            write_disabled_logged = false;
                        }
                        log::debug!(
                            "A1 LATENCY: mode={:?} ble_to_usb_write_us={}",
                            mode,
                            frame.ble_instant.elapsed().as_micros()
                        );
                    }
                    Err(embassy_usb::driver::EndpointError::Disabled) => {
                        // Expected when host has not completed enumeration or temporarily stops polling.
                        // Drop stale reports and wait for writes to succeed again.
                        if !write_disabled_logged {
                            log::warn!(
                                "USB IN endpoint disabled (host not ready/polling); dropping reports until re-enabled"
                            );
                            write_disabled_logged = true;
                        }
                    }
                    Err(embassy_usb::driver::EndpointError::BufferOverflow) => {
                        log::warn!("USB: buffer overflow (host not reading yet)");
                    }
                }
            }
        };

        let out_reader_fut = async {
            let mut buf = [0u8; 64];
            loop {
                // Wait until the host has completed SET_CONFIGURATION and the
                // endpoint is enabled before attempting reads.  Without this,
                // read() can return Disabled immediately at enumeration and the
                // future exits while device_fut is still running, leaving the
                // OUT endpoint permanently unserviced.
                read_ep.wait_enabled().await;
                log::info!("USB OUT endpoint enabled, listening for host output");
                loop {
                    match read_ep.read(&mut buf).await {
                        Ok(n) => {
                            #[cfg(feature = "trace-haptics")]
                            log::info!("USB OUT: {} bytes {:02x?}", n, &buf[..n.min(8)]);
                            #[cfg(not(feature = "trace-haptics"))]
                            log::info!("USB OUT: {} bytes {:02x?}", n, &buf[..n.min(8)]);
                            let frame = UsbOutFrame {
                                mode,
                                len: n as u8,
                                data: buf,
                            };
                            if USB_OUT_FRAMES.try_send(frame).is_err() {
                                log::warn!("USB OUT queue full, dropping host output frame");
                            }
                        }
                        Err(embassy_usb::driver::EndpointError::Disabled) => {
                            // Host reset or USB suspend — go back to wait_enabled.
                            break;
                        }
                        Err(e) => {
                            log::warn!("USB OUT read error: {:?}", e);
                        }
                    }
                }
            }
        };

        match select(
            embassy_futures::join::join(device_fut, embassy_futures::join::join(reporter_fut, out_reader_fut)),
            USB_REINIT.wait(),
        ).await {
            Either::First(_)  => log::warn!("USB device exited unexpectedly, reiniting…"),
            Either::Second(_) => log::info!("USB reinit requested; tearing down USB…"),
        }
        // usb_device + write_ep drop here, releasing the USB hardware.
        // Brief pause so the host registers the disconnect before we re-enumerate.
        embassy_time::Timer::after_millis(250).await;
    }
}

struct XInputControlHandler;

impl embassy_usb::Handler for XInputControlHandler {
    fn control_in<'a>(
        &'a mut self,
        req: embassy_usb::control::Request,
        buf: &'a mut [u8],
    ) -> Option<embassy_usb::control::InResponse<'a>> {
        if req.request_type == embassy_usb::control::RequestType::Vendor
            && req.recipient == embassy_usb::control::Recipient::Interface
            && req.request == 0x01
            && req.value == 0x0100
        {
            log::info!("Handling XInput magic message control request (len={})", req.length);
            let len = (req.length as usize).min(buf.len());
            for i in 0..len {
                buf[i] = 0;
            }
            Some(embassy_usb::control::InResponse::Accepted(&buf[..len]))
        } else {
            None
        }
    }
}

/// Handles HID class control requests for Switch and DualSense USB modes.
/// Most importantly responds to GET_DESCRIPTOR(type=0x22) so the host can read the
/// HID Report Descriptor — without this the device stalls during enumeration.
struct HidControlHandler {
    report_descriptor: &'static [u8],
}

/// DualSense needs stricter HID control responses than generic HID gamepads.
/// In particular, some hosts send GET_REPORT for feature reports and expect
/// the returned payload to start with the requested report ID.
struct DualSenseHidControlHandler {
    report_descriptor: &'static [u8],
}

impl embassy_usb::Handler for HidControlHandler {
    fn control_in<'a>(
        &'a mut self,
        req: embassy_usb::control::Request,
        buf: &'a mut [u8],
    ) -> Option<embassy_usb::control::InResponse<'a>> {
        // Standard GET_DESCRIPTOR for HID Report (bmRequestType=0x81, bRequest=0x06, wValueHigh=0x22)
        if req.request_type == embassy_usb::control::RequestType::Standard
            && req.recipient == embassy_usb::control::Recipient::Interface
            && req.request == 0x06
            && (req.value >> 8) as u8 == 0x22
        {
            let len = (req.length as usize).min(self.report_descriptor.len());
            Some(embassy_usb::control::InResponse::Accepted(&self.report_descriptor[..len]))
        // Some hosts issue HID class IN requests (GET_REPORT / GET_IDLE / GET_PROTOCOL)
        // during probing. Return zeroed payload instead of stalling to maximize compatibility.
        } else if req.request_type == embassy_usb::control::RequestType::Class
            && req.recipient == embassy_usb::control::Recipient::Interface
        {
            let len = (req.length as usize).min(buf.len());
            for b in &mut buf[..len] {
                *b = 0;
            }
            Some(embassy_usb::control::InResponse::Accepted(&buf[..len]))
        } else {
            None
        }
    }

    fn control_out(
        &mut self,
        req: embassy_usb::control::Request,
        _data: &[u8],
    ) -> Option<embassy_usb::control::OutResponse> {
        // Accept HID class requests: SET_IDLE (0x0A), SET_PROTOCOL (0x0B), SET_REPORT (0x09)
        if req.request_type == embassy_usb::control::RequestType::Class
            && req.recipient == embassy_usb::control::Recipient::Interface
        {
            Some(embassy_usb::control::OutResponse::Accepted)
        } else {
            None
        }
    }
}

impl embassy_usb::Handler for DualSenseHidControlHandler {
    fn control_in<'a>(
        &'a mut self,
        req: embassy_usb::control::Request,
        buf: &'a mut [u8],
    ) -> Option<embassy_usb::control::InResponse<'a>> {
        // Standard GET_DESCRIPTOR(HID Report)
        if req.request_type == embassy_usb::control::RequestType::Standard
            && req.recipient == embassy_usb::control::Recipient::Interface
            && req.request == 0x06
            && (req.value >> 8) as u8 == 0x22
        {
            let len = (req.length as usize).min(self.report_descriptor.len());
            return Some(embassy_usb::control::InResponse::Accepted(&self.report_descriptor[..len]));
        }

        // HID class IN requests.
        if req.request_type == embassy_usb::control::RequestType::Class
            && req.recipient == embassy_usb::control::Recipient::Interface
        {
            let len = (req.length as usize).min(buf.len());
            if len == 0 {
                return Some(embassy_usb::control::InResponse::Accepted(&[]));
            }

            match req.request {
                // GET_REPORT: wValue hi=report type, lo=report id.
                0x01 => {
                    let report_id = (req.value & 0xFF) as u8;
                    let report_type = ((req.value >> 8) & 0xFF) as u8;
                    for b in &mut buf[..len] {
                        *b = 0;
                    }
                    // For numbered reports, echo requested ID in first byte.
                    // DualSense hosts commonly require this for feature reports.
                    if report_type == 0x03 || report_type == 0x01 || report_type == 0x02 {
                        buf[0] = report_id;
                    }
                    if report_type == 0x03 { // Feature Report
                        match report_id {
                            0x09 => {
                                // MAC address (pairing info). Set a valid MAC at buf[1..7]
                                if len >= 7 {
                                    buf[1..7].copy_from_slice(&[0x00, 0x11, 0x22, 0x33, 0x44, 0x55]);
                                }
                            }
                            0x20 => {
                                // Firmware & hardware versions
                                if len >= 48 {
                                    // hw_version at offset 24
                                    buf[24..28].copy_from_slice(&[0x01, 0x00, 0x00, 0x00]);
                                    // fw_version at offset 28
                                    buf[28..32].copy_from_slice(&[0x02, 0x00, 0x00, 0x00]);
                                    // update_version at offset 44
                                    buf[44..48].copy_from_slice(&[0x03, 0x00, 0x00, 0x00]);
                                }
                            }
                            _ => {}
                        }
                    }
                    Some(embassy_usb::control::InResponse::Accepted(&buf[..len]))
                }
                // GET_IDLE
                0x02 => {
                    buf[0] = 0;
                    Some(embassy_usb::control::InResponse::Accepted(&buf[..1.min(len)]))
                }
                // GET_PROTOCOL: report protocol
                0x03 => {
                    buf[0] = 1;
                    Some(embassy_usb::control::InResponse::Accepted(&buf[..1.min(len)]))
                }
                _ => {
                    for b in &mut buf[..len] {
                        *b = 0;
                    }
                    Some(embassy_usb::control::InResponse::Accepted(&buf[..len]))
                }
            }
        } else {
            None
        }
    }

    fn control_out(
        &mut self,
        req: embassy_usb::control::Request,
        _data: &[u8],
    ) -> Option<embassy_usb::control::OutResponse> {
        if req.request_type == embassy_usb::control::RequestType::Class
            && req.recipient == embassy_usb::control::Recipient::Interface
        {
            // SET_REPORT (0x09)
            if req.request == 0x09 {
                let report_id = (req.value & 0xFF) as u8;
                let report_type = ((req.value >> 8) & 0xFF) as u8;
                if report_type == 0x02 && report_id == 0x02 {
                    let mut buf = [0u8; 64];
                    if !_data.is_empty() && _data[0] == 0x02 {
                        let len = _data.len().min(64);
                        buf[..len].copy_from_slice(&_data[..len]);
                    } else {
                        buf[0] = 0x02;
                        let len = _data.len().min(63);
                        buf[1..1+len].copy_from_slice(&_data[..len]);
                    }
                    let frame = UsbOutFrame {
                        mode: crate::storage::OutputMode::DualSense,
                        len: 64,
                        data: buf,
                    };
                    if USB_OUT_FRAMES.try_send(frame).is_err() {
                        log::warn!("USB OUT queue full from control_out, dropping DualSense frame");
                    }
                }
            }
            Some(embassy_usb::control::OutResponse::Accepted)
        } else {
            None
        }
    }
}

// Module-level handler storage reused across USB reinits.
// Safety: only setup_usb writes these; previous UsbDevice is always dropped first.
static mut XINPUT_HANDLER_STORAGE: XInputControlHandler = XInputControlHandler;
static mut SWITCH_HID_HANDLER_STORAGE: HidControlHandler = HidControlHandler { report_descriptor: &[] };
static mut DUALSENSE_HANDLER_STORAGE: DualSenseHidControlHandler = DualSenseHidControlHandler { report_descriptor: &[] };

#[allow(static_mut_refs)] // Safety: caller (usb_manager_task) ensures previous UsbDevice is dropped first
pub fn setup_usb(
    driver: EspUsbDriver<'static>,
    config_descriptor_buf: &'static mut [u8],
    bos_descriptor_buf: &'static mut [u8],
    msos_descriptor_buf: &'static mut [u8],
    control_buf: &'static mut [u8],
    mode: crate::storage::OutputMode,
) -> (
    UsbDevice<'static, EspUsbDriver<'static>>,
    <EspUsbDriver<'static> as embassy_usb::driver::Driver<'static>>::EndpointIn,
    <EspUsbDriver<'static> as embassy_usb::driver::Driver<'static>>::EndpointOut,
) {
    match mode {
        crate::storage::OutputMode::XInput => {
            log::info!("Setting up USB as Xbox 360 (XInput) Controller...");
            // Xbox 360 controller device configuration
            let mut config = Config::new(0x045E, 0x028E);
            config.manufacturer = Some("Microsoft Corp.");
            config.product = Some("Xbox 360 Wireless Controller");
            config.serial_number = Some("0000001");
            config.device_class = 0xFF;
            config.device_sub_class = 0xFF;
            config.device_protocol = 0xFF;
            config.composite_with_iads = false;  // Single-function device, no IADs needed
            config.max_packet_size_0 = 64;

            let mut builder = Builder::new(
                driver,
                config,
                config_descriptor_buf,
                bos_descriptor_buf,
                msos_descriptor_buf,
                control_buf,
            );

            // Safety: previous UsbDevice is dropped before each reinit.
            unsafe {
                XINPUT_HANDLER_STORAGE = XInputControlHandler;
                builder.handler(&mut XINPUT_HANDLER_STORAGE);
            }

            let write_ep;
            let read_ep;
            {
                // XInput Interface 0: Class 0xFF, SubClass 0x5D, Protocol 0x01
                let mut function = builder.function(0xFF, 0x5D, 0x01);
                let mut interface = function.interface();
                let mut alt = interface.alt_setting(0xFF, 0x5D, 0x01, None);

                // Xbox 360 controller class-specific descriptor (17 bytes total)
                alt.descriptor(
                    0x21,
                    &[
                        0x00, 0x01, 0x01, 0x25, 0x81, 0x14, 0x00, 0x00, 0x00, 0x00, 0x13, 0x01, 0x08, 0x00,
                        0x00,
                    ],
                );

                // Add endpoints:
                // Interrupt IN endpoint (Endpoint 1 IN) - 1ms poll rate
                write_ep = alt.endpoint_interrupt_in(None, 32, 1);
                // Interrupt OUT endpoint (Endpoint 2 OUT) - 4ms poll rate
                read_ep = alt.endpoint_interrupt_out(None, 32, 4);
            }

            let usb_device = builder.build();
            (usb_device, write_ep, read_ep)
        }
        crate::storage::OutputMode::SwitchPro => {
            log::info!("Setting up USB as Hori Pokken Switch Controller...");
            // Hori Pokken Controller / Nintendo Switch controller configuration
            let mut config = Config::new(0x0F0D, 0x0092); // Pokken Tournament Controller (0x0F0D:0x0092)
            config.manufacturer = Some("Nintendo Co., Ltd.");
            config.product = Some("Pro Controller");
            config.serial_number = None;
            config.device_class = 0x00;
            config.device_sub_class = 0x00;
            config.device_protocol = 0x00;
            config.composite_with_iads = false;
            config.max_packet_size_0 = 64;

            let mut builder = Builder::new(
                driver,
                config,
                config_descriptor_buf,
                bos_descriptor_buf,
                msos_descriptor_buf,
                control_buf,
            );

            let switch_hid_desc: &'static [u8];
            let write_ep;
            let read_ep;
            {
                // Switch HID Interface 0: Class 0x03 (HID), SubClass 0x00, Protocol 0x00
                let mut function = builder.function(0x03, 0x00, 0x00);
                let mut interface = function.interface();
                let mut alt = interface.alt_setting(0x03, 0x00, 0x00, None);

                // Pokken / HORI Switch controller HID Report Descriptor
                // Size = 60 bytes
                const SWITCH_REPORT_DESC: &[u8] = &[
                    0x05, 0x01,        // Usage Page (Generic Desktop Ctrls)
                    0x09, 0x05,        // Usage (Game Pad)
                    0xA1, 0x01,        // Collection (Application)
                    // Buttons (2 bytes / 16 buttons)
                    0x15, 0x00,        //   Logical Minimum (0)
                    0x25, 0x01,        //   Logical Maximum (1)
                    0x35, 0x00,        //   Physical Minimum (0)
                    0x45, 0x01,        //   Physical Maximum (1)
                    0x75, 0x01,        //   Report Size (1)
                    0x95, 0x10,        //   Report Count (16)
                    0x05, 0x09,        //   Usage Page (Button)
                    0x19, 0x01,        //   Usage Minimum (0x01)
                    0x29, 0x10,        //   Usage Maximum (0x10)
                    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
                    // HAT Switch (1 nibble, 0-7)
                    0x05, 0x01,        //   Usage Page (Generic Desktop Ctrls)
                    0x25, 0x07,        //   Logical Maximum (7)
                    0x46, 0x3B, 0x01,  //   Physical Maximum (315)
                    0x75, 0x04,        //   Report Size (4)
                    0x95, 0x01,        //   Report Count (1)
                    0x65, 0x14,        //   Unit (English Rotational, Centimeter)
                    0x09, 0x39,        //   Usage (Hat switch)
                    0x81, 0x42,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,Null State)
                    // D-pad padding/unused (1 nibble)
                    0x65, 0x00,        //   Unit (None)
                    0x95, 0x01,        //   Report Count (1)
                    0x81, 0x01,        //   Input (Const,Array,Abs,No Wrap,Linear,Preferred State,No Null Position)
                    // Joysticks (4 bytes / 4 axes: LX, LY, RX, RY)
                    0x26, 0xFF, 0x00,  //   Logical Maximum (255)
                    0x46, 0xFF, 0x00,  //   Physical Maximum (255)
                    0x09, 0x30,        //   Usage (X)
                    0x09, 0x31,        //   Usage (Y)
                    0x09, 0x32,        //   Usage (Z)
                    0x09, 0x35,        //   Usage (Rz)
                    0x75, 0x08,        //   Report Size (8)
                    0x95, 0x04,        //   Report Count (4)
                    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
                    // Vendor Specific (1 byte)
                    0x06, 0x00, 0xFF,  //   Usage Page (Vendor Defined 0xFF00)
                    0x09, 0x20,        //   Usage (0x20)
                    0x95, 0x01,        //   Report Count (1)
                    0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
                    // Output (8 bytes)
                    0x0A, 0x21, 0x26,  //   Usage (0x2621)
                    0x95, 0x08,        //   Report Count (8)
                    0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
                    0xC0,              // End Collection
                ];

                // alt.descriptor(0x21, payload) prepends [payload.len()+2, 0x21] automatically.
                // So payload must be the 7-byte content only (no bLength / bDescriptorType).
                let len = SWITCH_REPORT_DESC.len() as u16;
                let class_payload = [0x11u8, 0x01, 0x00, 0x01, 0x22, (len & 0xFF) as u8, (len >> 8) as u8];
                switch_hid_desc = SWITCH_REPORT_DESC;
                alt.descriptor(0x21, &class_payload);
                // Report descriptor is NOT embedded in config; it is returned via GET_DESCRIPTOR
                // control request handled by SWITCH_HID_HANDLER_STORAGE below.

                // Add endpoints: IN endpoint 1 (8 bytes, 1ms interval), OUT endpoint 2 (8 bytes, 1ms interval)
                write_ep = alt.endpoint_interrupt_in(None, 8, 1);
                read_ep = alt.endpoint_interrupt_out(None, 8, 1);
            }

            // Safety: previous UsbDevice is dropped before each reinit.
            unsafe {
                SWITCH_HID_HANDLER_STORAGE = HidControlHandler { report_descriptor: switch_hid_desc };
                builder.handler(&mut SWITCH_HID_HANDLER_STORAGE);
            }

            let usb_device = builder.build();
            (usb_device, write_ep, read_ep)
        }
        crate::storage::OutputMode::DualSense => {
            log::info!("Setting up USB as Sony DualSense (PS5) Controller...");
            let mut config = Config::new(0x054C, 0x0CE6); // Sony (0x054C) DualSense (0x0CE6)
            config.manufacturer = Some("Sony Interactive Entertainment");
            config.product = Some("Wireless Controller");
            config.serial_number = None;
            config.device_class = 0x00;
            config.device_sub_class = 0x00;
            config.device_protocol = 0x00;
            config.composite_with_iads = false;
            config.max_packet_size_0 = 64;

            let mut builder = Builder::new(
                driver,
                config,
                config_descriptor_buf,
                bos_descriptor_buf,
                msos_descriptor_buf,
                control_buf,
            );

            let dualsense_hid_desc: &'static [u8];
            let write_ep;
            let read_ep;
            {
                // HID Interface 0: Class 0x03, SubClass 0x00, Protocol 0x00
                let mut function = builder.function(0x03, 0x00, 0x00);
                let mut interface = function.interface();
                let mut alt = interface.alt_setting(0x03, 0x00, 0x00, None);

                // Keep this descriptor below 255 bytes: embassy-usb writes a u8 bLength for each
                // descriptor entry and panics if any single descriptor exceeds that size.
                const DUALSENSE_REPORT_DESC: &[u8] = &[
                    0x05, 0x01,             // Usage Page (Generic Desktop)
                    0x09, 0x05,             // Usage (Game Pad)
                    0xA1, 0x01,             // Collection (Application)
                    // Report ID 1: 64-byte input (1 byte ID + 63 bytes payload)
                    0x85, 0x01,
                    // Bytes 1-6: LX, LY, RX, RY, L2, R2 axes
                    0x15, 0x00,
                    0x26, 0xFF, 0x00,
                    0x75, 0x08,
                    0x95, 0x06,
                    0x09, 0x30, 0x09, 0x31, 0x09, 0x32,
                    0x09, 0x35, 0x09, 0x33, 0x09, 0x34,
                    0x81, 0x02,             // Input (Data,Var,Abs)
                    // Byte 7: frame counter (vendor constant)
                    0x06, 0x00, 0xFF,
                    0x09, 0x20,
                    0x75, 0x08,
                    0x95, 0x01,
                    0x81, 0x03,             // Input (Const) — counter byte
                    // Byte 8 low nibble: hat switch
                    0x05, 0x01,
                    0x09, 0x39,
                    0x15, 0x00,
                    0x25, 0x07,
                    0x35, 0x00,
                    0x46, 0x3B, 0x01,
                    0x65, 0x14,
                    0x75, 0x04,
                    0x95, 0x01,
                    0x81, 0x42,             // Input (Data,Var,Abs,NullState)
                    0x65, 0x00,
                    // Byte 8 high nibble: Square, Cross, Circle, Triangle (buttons 1-4)
                    0x05, 0x09,
                    0x19, 0x01,
                    0x29, 0x04,
                    0x15, 0x00,
                    0x25, 0x01,
                    0x75, 0x01,
                    0x95, 0x04,
                    0x81, 0x02,             // Input (Data,Var,Abs)
                    // Byte 9: L1,R1,L2,R2,Share,Options,L3,R3 (buttons 5-12)
                    0x19, 0x05,
                    0x29, 0x0C,
                    0x95, 0x08,
                    0x81, 0x02,             // Input (Data,Var,Abs)
                    // Byte 10 low 2 bits: PS, Touchpad (buttons 13-14)
                    0x19, 0x0D,
                    0x29, 0x0E,
                    0x95, 0x02,
                    0x81, 0x02,             // Input (Data,Var,Abs)
                    // Byte 10 high 6 bits: padding
                    0x75, 0x06,
                    0x95, 0x01,
                    0x81, 0x03,             // Input (Const)
                    // Bytes 11-63: vendor data (IMU, touchpad, etc.) = 53 bytes
                    0x06, 0x00, 0xFF,
                    0x09, 0x21,
                    0x75, 0x08,
                    0x95, 0x35,
                    0x81, 0x02,             // Input (Data,Var,Abs)
                    // Report ID 2: 64-byte output report (1 byte ID + 63 bytes)
                    0x85, 0x02,
                    0x06, 0x00, 0xFF,
                    0x09, 0x22,
                    0x75, 0x08,
                    0x95, 0x3F,
                    0x91, 0x02,             // Output (Data,Var,Abs)
                    0xC0,
                ];

                let len = DUALSENSE_REPORT_DESC.len() as u16;
                let class_payload = [0x11u8, 0x01, 0x00, 0x01, 0x22, (len & 0xFF) as u8, (len >> 8) as u8];
                dualsense_hid_desc = DUALSENSE_REPORT_DESC;
                alt.descriptor(0x21, &class_payload);
                // Report descriptor returned via GET_DESCRIPTOR control request only.

                // Add endpoints: IN endpoint 1 (64 bytes, 1ms interval), OUT endpoint 2 (64 bytes, 1ms interval)
                write_ep = alt.endpoint_interrupt_in(None, 64, 1);
                read_ep = alt.endpoint_interrupt_out(None, 64, 1);
            }

            // Safety: previous UsbDevice is dropped before each reinit.
            unsafe {
                DUALSENSE_HANDLER_STORAGE = DualSenseHidControlHandler { report_descriptor: dualsense_hid_desc };
                builder.handler(&mut DUALSENSE_HANDLER_STORAGE);
            }

            let usb_device = builder.build();
            (usb_device, write_ep, read_ep)
        }
    }
}

