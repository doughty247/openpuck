//! An in-memory stand-in for a Steam Controller 2's BLE GATT server,
//! implementing [`ControllerPeripheral`] instead of the real
//! `btleplug::api::Peripheral` trait (see `ble.rs`'s module doc for why).
//! Test-only: exercises the exact GATT shape and command sequence
//! documented in `docs/ble-protocol.md`, reconstructed from the
//! hardware-validated firmware (`src/bluetooth.rs` in the repo root) since
//! no real controller is available in this environment. It models what the
//! firmware's own characteristic enumeration logs show a real Steam
//! Controller 2 presenting: the Valve custom service (input + report
//! characteristics) plus a standard HID-over-GATT service exposing one
//! write-without-response-only characteristic (the output/rumble report)
//! and one readable+writable characteristic (the feature/settings report).

#![cfg(test)]

use crate::ble::{
    ControllerPeripheral, D0G_INPUT_UUID, HID_SERVICE_UUID, TRITON_INPUT_UUID, VALVE_REPORT_UUID, VALVE_SERVICE_UUID,
};
use anyhow::Result;
use btleplug::api::{BDAddr, CharPropFlags, Characteristic, PeripheralProperties, ValueNotification, WriteType};
use futures::stream::Stream;
use std::collections::BTreeSet;
use std::pin::Pin;
use std::sync::{Arc, Mutex};
use tokio::sync::mpsc;
use tokio_stream::wrappers::UnboundedReceiverStream;
use uuid::Uuid;

/// A single recorded `write()` call, for asserting the exact command bytes
/// a test scenario sent.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RecordedWrite {
    pub characteristic_uuid: Uuid,
    pub data: Vec<u8>,
    pub write_type: WriteType,
}

struct Inner {
    writes: Vec<RecordedWrite>,
    notify_tx: Option<mpsc::UnboundedSender<ValueNotification>>,
}

/// Mock Steam Controller 2 GATT server. Cheap to `Clone` (shares state via
/// `Arc`), matching how `btleplug::platform::Peripheral` handles are shared.
#[derive(Clone)]
pub struct MockController {
    name: String,
    characteristics: BTreeSet<Characteristic>,
    inner: Arc<Mutex<Inner>>,
}

fn char_flags(read: bool, write: bool, write_no_resp: bool, notify: bool) -> CharPropFlags {
    let mut flags = CharPropFlags::empty();
    if read { flags |= CharPropFlags::READ; }
    if write { flags |= CharPropFlags::WRITE; }
    if write_no_resp { flags |= CharPropFlags::WRITE_WITHOUT_RESPONSE; }
    if notify { flags |= CharPropFlags::NOTIFY; }
    flags
}

fn characteristic(service_uuid: Uuid, uuid: Uuid, props: CharPropFlags) -> Characteristic {
    Characteristic { uuid, service_uuid, properties: props, descriptors: BTreeSet::new() }
}

impl MockController {
    /// A Steam Controller 2 (Triton) with the standard GATT layout: Valve
    /// input (notify) + report characteristics, plus a HID service with a
    /// write-without-response-only output characteristic and a
    /// readable+writable feature characteristic — exactly the property
    /// combination `ble::score_hid_characteristics` is designed to pick apart.
    pub fn steam_controller_2(name: &str) -> Self {
        let mut characteristics = BTreeSet::new();
        characteristics.insert(characteristic(
            VALVE_SERVICE_UUID,
            TRITON_INPUT_UUID,
            char_flags(false, false, false, true),
        ));
        characteristics.insert(characteristic(
            VALVE_SERVICE_UUID,
            VALVE_REPORT_UUID,
            char_flags(true, true, true, false),
        ));
        // Real HID-over-GATT devices commonly expose *multiple* Report
        // characteristics (UUID 0x2A4D) within one service, distinguished at
        // the ATT layer by declaration handle and a Report Reference
        // descriptor rather than by UUID — btleplug doesn't surface either,
        // which is exactly why `score_hid_characteristics` scores by
        // properties instead of relying on a unique UUID per report. Model
        // that faithfully: both HID characteristics below share UUID 0x2A4D
        // and are told apart only by their properties, same as in production.
        const HID_REPORT_UUID: Uuid = uuid::uuid!("00002a4d-0000-1000-8000-00805f9b34fb");
        // HID output/rumble report: write-without-response, not readable —
        // the property combination score_hid_characteristics treats as "output".
        characteristics.insert(characteristic(HID_SERVICE_UUID, HID_REPORT_UUID, char_flags(false, false, true, false)));
        // HID feature/settings report: readable + write-with-response — the
        // combination score_hid_characteristics treats as "feature".
        characteristics.insert(characteristic(HID_SERVICE_UUID, HID_REPORT_UUID, char_flags(true, true, false, false)));

        MockController {
            name: name.to_string(),
            characteristics,
            inner: Arc::new(Mutex::new(Inner { writes: Vec::new(), notify_tx: None })),
        }
    }

    /// A Gen 1 (2015, "D0G") Steam Controller: only the old input
    /// characteristic, no Triton input. `ble::connect_and_discover` should
    /// reject this — this bridge only supports Steam Controller 2.
    pub fn gen1_steam_controller(name: &str) -> Self {
        let mut characteristics = BTreeSet::new();
        characteristics.insert(characteristic(VALVE_SERVICE_UUID, D0G_INPUT_UUID, char_flags(false, false, false, true)));
        MockController {
            name: name.to_string(),
            characteristics,
            inner: Arc::new(Mutex::new(Inner { writes: Vec::new(), notify_tx: None })),
        }
    }

    pub fn advertised_name(&self) -> &str {
        &self.name
    }

    /// All writes recorded so far, in call order.
    pub fn recorded_writes(&self) -> Vec<RecordedWrite> {
        self.inner.lock().unwrap().writes.clone()
    }

    /// Pushes a synthetic input notification as if the real controller sent
    /// it. Panics if `notifications()` hasn't been called yet (matches real
    /// BLE semantics: you must subscribe/open the stream before events flow).
    pub fn push_input_report(&self, payload: &[u8]) {
        let inner = self.inner.lock().unwrap();
        let tx = inner.notify_tx.as_ref().expect("push_input_report called before notifications() was opened");
        tx.send(ValueNotification { uuid: TRITON_INPUT_UUID, value: payload.to_vec() })
            .expect("notification receiver dropped");
    }
}

impl ControllerPeripheral for MockController {
    fn address(&self) -> BDAddr {
        BDAddr::default()
    }

    async fn properties(&self) -> Result<Option<PeripheralProperties>> {
        Ok(Some(PeripheralProperties { local_name: Some(self.name.clone()), ..Default::default() }))
    }

    async fn connect(&self) -> Result<()> {
        Ok(())
    }

    async fn discover_services(&self) -> Result<()> {
        Ok(())
    }

    fn characteristics(&self) -> BTreeSet<Characteristic> {
        self.characteristics.clone()
    }

    async fn subscribe(&self, characteristic: &Characteristic) -> Result<()> {
        if !self.characteristics.contains(characteristic) {
            anyhow::bail!("subscribe to unknown characteristic {}", characteristic.uuid);
        }
        Ok(())
    }

    async fn write(&self, characteristic: &Characteristic, data: &[u8], write_type: WriteType) -> Result<()> {
        if !self.characteristics.contains(characteristic) {
            anyhow::bail!("write to unknown characteristic {}", characteristic.uuid);
        }
        self.inner.lock().unwrap().writes.push(RecordedWrite {
            characteristic_uuid: characteristic.uuid,
            data: data.to_vec(),
            write_type,
        });
        Ok(())
    }

    async fn notifications(&self) -> Result<Pin<Box<dyn Stream<Item = ValueNotification> + Send>>> {
        let (tx, rx) = mpsc::unbounded_channel();
        self.inner.lock().unwrap().notify_tx = Some(tx);
        Ok(Box::pin(UnboundedReceiverStream::new(rx)))
    }
}
