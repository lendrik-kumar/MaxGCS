// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// macOS HID backend — IOKit HID Manager (C API declared below, linked against the IOKit framework).
// Reads a device's RAW HID input elements each tick (axes / hat / buttons keyed by usage page +
// usage), with NO gamepad-model projection — matching the Windows (WGI) and Linux (evdev) backends so
// flight hardware / RC transmitters map correctly. See docs/archive/MSP_RC_CONTROL.md §6.
//
// We poll element values with IOHIDDeviceGetValue (which reads the device's current state) rather than
// scheduling the manager on a CFRunLoop: the owning input thread already ticks at ~50 Hz, so a direct
// per-tick read gives the same live-state view the other backends produce, with no event queue to
// drain. Element metadata is enumerated once per (throttled) rescan and cached; only the cheap value
// reads happen on the 20 ms snapshot path.
//
// NOTE: compiled only on macOS; not built / verified on the Windows or Linux hosts. The first HID
// access may raise a one-time "Input Monitoring" permission prompt on recent macOS; until granted the
// device list is empty (IOHIDManagerCopyDevices returns nothing).

use std::collections::HashMap;
use std::os::raw::c_void;
use std::ptr;
use std::time::{Duration, Instant};

use core_foundation::array::CFArray;
use core_foundation::base::{CFType, TCFType};
use core_foundation::dictionary::CFDictionary;
use core_foundation::number::CFNumber;
use core_foundation::string::CFString;

use core_foundation_sys::array::{CFArrayGetCount, CFArrayGetValueAtIndex, CFArrayRef};
use core_foundation_sys::base::{CFGetTypeID, CFIndex, CFRelease, CFRetain, CFTypeRef};
use core_foundation_sys::dictionary::CFDictionaryRef;
use core_foundation_sys::number::{CFNumberGetTypeID, CFNumberRef};
use core_foundation_sys::set::{CFSetGetCount, CFSetGetValues, CFSetRef};
use core_foundation_sys::string::CFStringRef;

use super::{HidAxis, HidButton, HidDevice, HidHat, HidSnapshot};

const RESCAN_INTERVAL: Duration = Duration::from_millis(1000);

// ── HID usage pages / usages we care about (USB HID Usage Tables) ────────────────────────────────
const PAGE_GENERIC_DESKTOP: u32 = 0x01;
const PAGE_SIMULATION: u32 = 0x02; // throttle / rudder / accelerator etc. on many flight controllers
const PAGE_BUTTON: u32 = 0x09;
/// Generic-Desktop axis usages: X, Y, Z, Rx, Ry, Rz, Slider, Dial, Wheel.
const AXIS_USAGES: std::ops::RangeInclusive<u32> = 0x30..=0x38;
const USAGE_HAT_SWITCH: u32 = 0x39;

// ── IOHIDElementType values (IOKit/hid/IOHIDKeys.h) ──────────────────────────────────────────────
const ELEM_INPUT_MISC: u32 = 1;
const ELEM_INPUT_BUTTON: u32 = 2;
const ELEM_INPUT_AXIS: u32 = 3;

type IOHIDManagerRef = *mut c_void;
type IOHIDDeviceRef = *mut c_void;
type IOHIDElementRef = *mut c_void;
type IOHIDValueRef = *mut c_void;
type IOReturn = i32;
type IOOptionBits = u32;

const K_IO_RETURN_SUCCESS: IOReturn = 0;

#[link(name = "IOKit", kind = "framework")]
extern "C" {
    fn IOHIDManagerCreate(allocator: *const c_void, options: IOOptionBits) -> IOHIDManagerRef;
    fn IOHIDManagerSetDeviceMatchingMultiple(manager: IOHIDManagerRef, multiple: CFArrayRef);
    fn IOHIDManagerOpen(manager: IOHIDManagerRef, options: IOOptionBits) -> IOReturn;
    fn IOHIDManagerClose(manager: IOHIDManagerRef, options: IOOptionBits) -> IOReturn;
    fn IOHIDManagerCopyDevices(manager: IOHIDManagerRef) -> CFSetRef;
    fn IOHIDDeviceGetProperty(device: IOHIDDeviceRef, key: CFStringRef) -> CFTypeRef;
    fn IOHIDDeviceCopyMatchingElements(
        device: IOHIDDeviceRef,
        matching: CFDictionaryRef,
        options: IOOptionBits,
    ) -> CFArrayRef;
    fn IOHIDDeviceGetValue(
        device: IOHIDDeviceRef,
        element: IOHIDElementRef,
        p_value: *mut IOHIDValueRef,
    ) -> IOReturn;
    fn IOHIDElementGetType(element: IOHIDElementRef) -> u32;
    fn IOHIDElementGetUsagePage(element: IOHIDElementRef) -> u32;
    fn IOHIDElementGetUsage(element: IOHIDElementRef) -> u32;
    fn IOHIDElementGetLogicalMin(element: IOHIDElementRef) -> CFIndex;
    fn IOHIDElementGetLogicalMax(element: IOHIDElementRef) -> CFIndex;
    fn IOHIDValueGetIntegerValue(value: IOHIDValueRef) -> CFIndex;
}

/// One cached input element (device ref + element ref are CFRetained for the entry's lifetime).
struct AxisElem {
    code: u32,
    element: IOHIDElementRef,
    min: i64,
    max: i64,
}

struct HatElem {
    code: u32,
    element: IOHIDElementRef,
    min: i64,
    max: i64,
}

struct ButtonElem {
    code: u32,
    element: IOHIDElementRef,
}

struct DeviceEntry {
    id: usize,
    device: IOHIDDeviceRef, // CFRetained
    name: String,
    uuid: String,
    axes: Vec<AxisElem>,
    hats: Vec<HatElem>,
    buttons: Vec<ButtonElem>,
}

impl DeviceEntry {
    /// Release every IOKit ref this entry retained (its cached elements + the device itself).
    fn release(&self) {
        unsafe {
            for a in &self.axes {
                CFRelease(a.element as CFTypeRef);
            }
            for h in &self.hats {
                CFRelease(h.element as CFTypeRef);
            }
            for b in &self.buttons {
                CFRelease(b.element as CFTypeRef);
            }
            CFRelease(self.device as CFTypeRef);
        }
    }
}

pub struct IOKitBackend {
    manager: IOHIDManagerRef,
    devices: Vec<DeviceEntry>,
    ids: HashMap<String, usize>,
    next_id: usize,
    last_scan: Option<Instant>,
}

impl IOKitBackend {
    pub fn new() -> Self {
        let manager = unsafe { IOHIDManagerCreate(ptr::null(), 0) };
        if !manager.is_null() {
            let matching = matching_array();
            unsafe {
                IOHIDManagerSetDeviceMatchingMultiple(manager, matching.as_concrete_TypeRef());
                let r = IOHIDManagerOpen(manager, 0);
                if r != K_IO_RETURN_SUCCESS {
                    // Most commonly this is the pending "Input Monitoring" grant; poll() will simply
                    // return an empty list until the user allows it (or a device appears).
                    log::warn!("IOHIDManagerOpen failed (IOReturn {r}); HID input unavailable until granted");
                }
            }
        } else {
            log::warn!("IOHIDManagerCreate returned null; HID input disabled");
        }
        Self {
            manager,
            devices: Vec::new(),
            ids: HashMap::new(),
            next_id: 0,
            last_scan: None,
        }
    }

    fn rescan(&mut self) {
        if self.manager.is_null() {
            return;
        }
        let set = unsafe { IOHIDManagerCopyDevices(self.manager) };
        if set.is_null() {
            self.clear_devices();
            return;
        }

        // Snapshot the borrowed device refs out of the returned CFSet, then release the set.
        let count = unsafe { CFSetGetCount(set) };
        let mut raw: Vec<*const c_void> = vec![ptr::null(); count.max(0) as usize];
        unsafe {
            CFSetGetValues(set, raw.as_mut_ptr());
            CFRelease(set as CFTypeRef);
        }

        // Rebuild from scratch each rescan (≤1 Hz): release the old cached refs first.
        self.clear_devices();

        let mut entries = Vec::with_capacity(raw.len());
        for &v in &raw {
            let device = v as IOHIDDeviceRef;
            if device.is_null() {
                continue;
            }

            let (axes, hats, buttons) = enumerate_elements(device);
            if axes.is_empty() && hats.is_empty() && buttons.is_empty() {
                continue; // nothing usable (release nothing — enumerate_elements retains only kept ones)
            }

            let vid = device_prop_i32(device, "VendorID").unwrap_or(0);
            let pid = device_prop_i32(device, "ProductID").unwrap_or(0);
            let uuid = device_prop_string(device, "SerialNumber")
                .filter(|s| !s.is_empty())
                .map(|s| format!("{vid:04x}:{pid:04x}:{s}"))
                .unwrap_or_else(|| {
                    let loc = device_prop_i32(device, "LocationID").unwrap_or(0);
                    format!("{vid:04x}:{pid:04x}:{loc:08x}")
                });
            let name = device_prop_string(device, "Product")
                .filter(|s| !s.is_empty())
                .unwrap_or_else(|| format!("Joystick {vid:04x}:{pid:04x}"));

            let id = *self.ids.entry(uuid.clone()).or_insert_with(|| {
                let id = self.next_id;
                self.next_id += 1;
                id
            });

            // Retain the device so the ref stays valid until this entry is released.
            unsafe { CFRetain(device as CFTypeRef) };
            entries.push(DeviceEntry { id, device, name, uuid, axes, hats, buttons });
        }

        // Stable order so the device list doesn't reshuffle between rescans.
        entries.sort_by_key(|e| e.id);
        self.devices = entries;
    }

    fn clear_devices(&mut self) {
        for d in &self.devices {
            d.release();
        }
        self.devices.clear();
    }
}

impl super::HidBackend for IOKitBackend {
    fn poll(&mut self) -> Vec<HidDevice> {
        let due = self.last_scan.map_or(true, |t| t.elapsed() >= RESCAN_INTERVAL);
        if due {
            self.rescan();
            self.last_scan = Some(Instant::now());
        }
        self.devices
            .iter()
            .map(|d| HidDevice {
                id: d.id,
                name: d.name.clone(),
                uuid: d.uuid.clone(),
                axes: d.axes.len(),
                buttons: d.buttons.len(),
                hats: d.hats.len(),
            })
            .collect()
    }

    fn snapshot(&mut self, id: usize) -> Option<HidSnapshot> {
        let dev = self.devices.iter().find(|d| d.id == id)?;

        let axes = dev
            .axes
            .iter()
            .filter_map(|a| {
                read_int(dev.device, a.element)
                    .map(|v| HidAxis { code: a.code, value: norm(v, a.min, a.max) })
            })
            .collect();

        let hats = dev
            .hats
            .iter()
            .filter_map(|h| {
                read_int(dev.device, h.element).map(|v| {
                    let (x, y) = hat_xy(v, h.min, h.max);
                    HidHat { code: h.code, x, y }
                })
            })
            .collect();

        let buttons = dev
            .buttons
            .iter()
            .filter_map(|b| {
                read_int(dev.device, b.element).map(|v| {
                    let pressed = v != 0;
                    HidButton { code: b.code, pressed, value: if pressed { 1.0 } else { 0.0 } }
                })
            })
            .collect();

        Some(HidSnapshot { id, axes, buttons, hats })
    }
}

impl Drop for IOKitBackend {
    fn drop(&mut self) {
        self.clear_devices();
        if !self.manager.is_null() {
            unsafe {
                IOHIDManagerClose(self.manager, 0);
                CFRelease(self.manager as CFTypeRef);
            }
        }
    }
}

/// Build the device-matching array: Generic-Desktop Joystick (0x04), Game Pad (0x05) and
/// Multi-axis Controller (0x08) — the three usages flight hardware / RC transmitters present as.
fn matching_array() -> CFArray<CFType> {
    let page_key = CFString::from_static_string("DeviceUsagePage");
    let usage_key = CFString::from_static_string("DeviceUsage");
    let dicts: Vec<CFType> = [0x04i64, 0x05, 0x08]
        .iter()
        .map(|&usage| {
            let d = CFDictionary::from_CFType_pairs(&[
                (page_key.as_CFType(), CFNumber::from(PAGE_GENERIC_DESKTOP as i64).as_CFType()),
                (usage_key.as_CFType(), CFNumber::from(usage).as_CFType()),
            ]);
            d.as_CFType()
        })
        .collect();
    CFArray::from_CFTypes(&dicts)
}

/// Enumerate a device's input elements once, classifying + CFRetaining the ones we stream. Codes are
/// stable per control: `(occurrence << 24) | (usage_page << 16) | usage` — the high byte disambiguates
/// the rare case of two controls sharing a usage, and the layout is stable across rescans because IOKit
/// returns elements in a fixed cookie order.
fn enumerate_elements(device: IOHIDDeviceRef) -> (Vec<AxisElem>, Vec<HatElem>, Vec<ButtonElem>) {
    let mut axes = Vec::new();
    let mut hats = Vec::new();
    let mut buttons = Vec::new();

    let arr = unsafe { IOHIDDeviceCopyMatchingElements(device, ptr::null(), 0) };
    if arr.is_null() {
        return (axes, hats, buttons);
    }

    let mut seen: HashMap<u32, u32> = HashMap::new();
    let mut code_for = |page: u32, usage: u32| -> u32 {
        let base = (page << 16) | (usage & 0xFFFF);
        let occ = seen.entry(base).or_insert(0);
        let code = (*occ << 24) | base;
        *occ += 1;
        code
    };

    let count = unsafe { CFArrayGetCount(arr) };
    for i in 0..count {
        let el = unsafe { CFArrayGetValueAtIndex(arr, i) } as IOHIDElementRef;
        if el.is_null() {
            continue;
        }
        let etype = unsafe { IOHIDElementGetType(el) };
        // Input elements only (skip Output/Feature/collections).
        if etype != ELEM_INPUT_MISC && etype != ELEM_INPUT_BUTTON && etype != ELEM_INPUT_AXIS {
            continue;
        }
        let page = unsafe { IOHIDElementGetUsagePage(el) };
        let usage = unsafe { IOHIDElementGetUsage(el) };

        let keep = |el: IOHIDElementRef| unsafe { CFRetain(el as CFTypeRef) };

        if page == PAGE_BUTTON && etype == ELEM_INPUT_BUTTON {
            keep(el);
            buttons.push(ButtonElem { code: code_for(page, usage), element: el });
        } else if page == PAGE_GENERIC_DESKTOP && usage == USAGE_HAT_SWITCH {
            let (min, max) = logical_range(el);
            keep(el);
            hats.push(HatElem { code: code_for(page, usage), element: el, min, max });
        } else if (page == PAGE_GENERIC_DESKTOP && AXIS_USAGES.contains(&usage))
            || page == PAGE_SIMULATION
        {
            let (min, max) = logical_range(el);
            keep(el);
            axes.push(AxisElem { code: code_for(page, usage), element: el, min, max });
        }
    }

    unsafe { CFRelease(arr as CFTypeRef) };
    (axes, hats, buttons)
}

fn logical_range(el: IOHIDElementRef) -> (i64, i64) {
    let min = unsafe { IOHIDElementGetLogicalMin(el) } as i64;
    let max = unsafe { IOHIDElementGetLogicalMax(el) } as i64;
    (min, max)
}

/// Read an element's current integer value (device state at call time). `None` if the read fails.
fn read_int(device: IOHIDDeviceRef, element: IOHIDElementRef) -> Option<i64> {
    let mut val: IOHIDValueRef = ptr::null_mut();
    let r = unsafe { IOHIDDeviceGetValue(device, element, &mut val) };
    if r != K_IO_RETURN_SUCCESS || val.is_null() {
        return None;
    }
    // The returned value is owned by the framework (Get semantics) — do not release.
    Some(unsafe { IOHIDValueGetIntegerValue(val) } as i64)
}

/// Map a raw axis reading to −1.0…+1.0 using its logical range (matches the Linux backend's `norm`).
fn norm(value: i64, minimum: i64, maximum: i64) -> f32 {
    let (min, max) = (minimum as f32, maximum as f32);
    if max > min {
        (2.0 * (value as f32 - min) / (max - min) - 1.0).clamp(-1.0, 1.0)
    } else {
        0.0
    }
}

/// Convert a HID hat-switch reading to 8-way (x, y) with +y = up. A value outside the logical range
/// is the standard "null"/centred state → (0, 0). Handles both 8-position (0.. .7) and 4-position
/// (0..3) hats; index 0 = up (North), increasing clockwise.
fn hat_xy(value: i64, min: i64, max: i64) -> (i32, i32) {
    if max <= min || value < min || value > max {
        return (0, 0);
    }
    let positions = (max - min + 1) as i64;
    let idx = value - min;
    match positions {
        8 => match idx {
            0 => (0, 1),   // N
            1 => (1, 1),   // NE
            2 => (1, 0),   // E
            3 => (1, -1),  // SE
            4 => (0, -1),  // S
            5 => (-1, -1), // SW
            6 => (-1, 0),  // W
            7 => (-1, 1),  // NW
            _ => (0, 0),
        },
        4 => match idx {
            0 => (0, 1),  // N
            1 => (1, 0),  // E
            2 => (0, -1), // S
            3 => (-1, 0), // W
            _ => (0, 0),
        },
        _ => (0, 0),
    }
}

// ── Device property helpers (IOHIDDeviceGetProperty follows Get rule — do not release the result) ──

fn device_prop_i32(device: IOHIDDeviceRef, key: &str) -> Option<i32> {
    let k = CFString::new(key);
    let raw = unsafe { IOHIDDeviceGetProperty(device, k.as_concrete_TypeRef()) };
    if raw.is_null() {
        return None;
    }
    unsafe {
        if CFGetTypeID(raw) != CFNumberGetTypeID() {
            return None;
        }
        CFNumber::wrap_under_get_rule(raw as CFNumberRef).to_i32()
    }
}

fn device_prop_string(device: IOHIDDeviceRef, key: &str) -> Option<String> {
    let k = CFString::new(key);
    let raw = unsafe { IOHIDDeviceGetProperty(device, k.as_concrete_TypeRef()) };
    if raw.is_null() {
        return None;
    }
    unsafe {
        if CFGetTypeID(raw) != CFString::type_id() {
            return None;
        }
        Some(CFString::wrap_under_get_rule(raw as CFStringRef).to_string())
    }
}
