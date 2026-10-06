// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// ArduPilot DataFlash .bin encoder — "blackbox" export for Mission Planner.
//
// The mirror image of ardupilot.rs: that module decodes DataFlash logs, this one writes them, so a
// flight recorded over any link (MSP/INAV included) can be opened in Mission Planner's log review,
// graphed, and shown on its map.
//
// The format is self-describing. A FMT record (type 128, fixed 89 bytes) registers one message
// schema — its type id, record length, 4-char name, format string and column labels — and every
// later record is decoded against the schema its type id names. So all FMT records go first, then
// the data. Every field encoder here is the exact inverse of `ardupilot::decode_field_value`,
// which is what makes the round-trip test in this file meaningful.
//
// What this cannot invent: a real FC's parameter dump, IMU/vibration data, or ArduPilot's internal
// controller targets. Those messages are simply absent rather than filled with zeros, so nothing
// in the file claims to be a measurement that was never taken. For an INAV flight the MODE records
// carry the closest ArduPilot mode number (see `ardu_mode_num`) and an MSG record preserves the
// original mode name, since INAV's modes have no exact ArduPilot equivalent.

use std::io::Write;
use std::path::Path;

use super::types::{Flight, TelemetryRecord};

// ─── Framing ─────────────────────────────────────────────────────────────────

const HDR0: u8 = 0xA3;
const HDR1: u8 = 0x95;
const FMT_TYPE_ID: u8 = 128;
/// FMT payload: type_id(1) + length(1) + name(4) + format(16) + labels(64).
const FMT_DATA_LEN: usize = 86;

/// One message schema we emit. `format` characters and `labels` are positionally aligned.
struct MsgDef {
    type_id: u8,
    name: &'static str,
    format: &'static str,
    labels: &'static str,
}

impl MsgDef {
    /// Total record length including the 3-byte header, as the FMT record must declare it.
    fn record_len(&self) -> u8 {
        (3 + payload_width(self.format)) as u8
    }
}

/// Byte width of a format string's payload. Mirrors `ardupilot::fmt_char_width`.
fn payload_width(format: &str) -> usize {
    format.bytes().map(char_width).sum()
}

fn char_width(c: u8) -> usize {
    match c as char {
        'b' | 'B' | 'M' => 1,
        'h' | 'H' | 'c' | 'C' => 2,
        'i' | 'I' | 'e' | 'E' | 'L' | 'f' => 4,
        'd' | 'q' | 'Q' => 8,
        'N' => 16,
        'Z' => 64,
        _ => 0,
    }
}

// ─── The schemas we write ────────────────────────────────────────────────────
//
// Type ids are ours to choose (the reader learns them from FMT); these follow the order ArduPilot
// itself tends to use. Names and field labels deliberately match real ArduPilot messages, because
// that is what Mission Planner looks for when it builds its graph tree and map.

const GPS: MsgDef = MsgDef {
    type_id: 130,
    name: "GPS",
    format: "QBIHBcLLeffffB",
    labels: "TimeUS,Status,GMS,GWk,NSats,HDop,Lat,Lng,Alt,Spd,GCrs,VZ,Yaw,U",
};

const ATT: MsgDef = MsgDef {
    type_id: 131,
    name: "ATT",
    format: "QccccCCCC",
    labels: "TimeUS,DesRoll,Roll,DesPitch,Pitch,DesYaw,Yaw,ErrRP,ErrYaw",
};

const BARO: MsgDef = MsgDef {
    type_id: 132,
    name: "BARO",
    format: "Qffc",
    labels: "TimeUS,Alt,CRt,Temp",
};

const BAT: MsgDef = MsgDef {
    type_id: 133,
    name: "BAT",
    format: "QBfffB",
    labels: "TimeUS,Instance,Volt,Curr,CurrTot,RemPct",
};

const MODE: MsgDef = MsgDef {
    type_id: 134,
    name: "MODE",
    format: "QMBB",
    labels: "TimeUS,Mode,ModeNum,Rsn",
};

const MSG: MsgDef = MsgDef {
    type_id: 135,
    name: "MSG",
    format: "QZ",
    labels: "TimeUS,Message",
};

const EV: MsgDef = MsgDef {
    type_id: 136,
    name: "EV",
    format: "QB",
    labels: "TimeUS,Id",
};

const ARSP: MsgDef = MsgDef {
    type_id: 137,
    name: "ARSP",
    format: "Qf",
    labels: "TimeUS,Airspeed",
};

const RCIN: MsgDef = MsgDef {
    type_id: 138,
    name: "RCIN",
    format: "QHHHH",
    labels: "TimeUS,C1,C2,C3,C4",
};

/// Emitted in this order; FMT records for all of them precede any data.
const ALL_DEFS: [&MsgDef; 9] = [&GPS, &ATT, &BARO, &BAT, &MODE, &MSG, &EV, &ARSP, &RCIN];

/// ArduPilot EV ids for arm and disarm, which is how its logs mark a flight.
const EV_ARMED: u8 = 10;
const EV_DISARMED: u8 = 11;

// ─── Payload builder ─────────────────────────────────────────────────────────

/// Accumulates one record's little-endian payload. Each method is the inverse of the matching arm
/// of `ardupilot::decode_field_value`, and every float→int conversion clamps rather than wrapping,
/// so a wild telemetry value cannot corrupt the record framing.
#[derive(Default)]
struct Payload {
    buf: Vec<u8>,
}

impl Payload {
    fn new() -> Self {
        Self { buf: Vec::with_capacity(64) }
    }

    /// 'B' / 'M' — u8.
    fn u8v(&mut self, v: u8) -> &mut Self {
        self.buf.push(v);
        self
    }

    /// 'H' — u16.
    fn u16v(&mut self, v: u16) -> &mut Self {
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// 'I' — u32.
    fn u32v(&mut self, v: u32) -> &mut Self {
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// 'Q' — u64.
    fn u64v(&mut self, v: u64) -> &mut Self {
        self.buf.extend_from_slice(&v.to_le_bytes());
        self
    }

    /// 'c' — i16 in hundredths.
    fn centi_i16(&mut self, v: f64) -> &mut Self {
        let scaled = (v * 100.0).round().clamp(i16::MIN as f64, i16::MAX as f64) as i16;
        self.buf.extend_from_slice(&scaled.to_le_bytes());
        self
    }

    /// 'C' — u16 in hundredths.
    fn centi_u16(&mut self, v: f64) -> &mut Self {
        let scaled = (v * 100.0).round().clamp(0.0, u16::MAX as f64) as u16;
        self.buf.extend_from_slice(&scaled.to_le_bytes());
        self
    }

    /// 'e' — i32 in hundredths.
    fn centi_i32(&mut self, v: f64) -> &mut Self {
        let scaled = (v * 100.0).round().clamp(i32::MIN as f64, i32::MAX as f64) as i32;
        self.buf.extend_from_slice(&scaled.to_le_bytes());
        self
    }

    /// 'L' — i32 degrees × 1e7.
    fn latlon(&mut self, deg: f64) -> &mut Self {
        let scaled = (deg * 1e7).round().clamp(i32::MIN as f64, i32::MAX as f64) as i32;
        self.buf.extend_from_slice(&scaled.to_le_bytes());
        self
    }

    /// 'f' — f32. NaN/inf would reach the reader as garbage, so they become 0.
    fn f32v(&mut self, v: f64) -> &mut Self {
        let f = if v.is_finite() { v as f32 } else { 0.0 };
        self.buf.extend_from_slice(&f.to_le_bytes());
        self
    }

    /// 'N' (16) / 'Z' (64) — fixed-width null-padded ASCII, truncated on a char boundary.
    fn text(&mut self, s: &str, width: usize) -> &mut Self {
        let mut bytes = [0u8; 64];
        let src = s.as_bytes();
        let n = src.len().min(width);
        bytes[..n].copy_from_slice(&src[..n]);
        self.buf.extend_from_slice(&bytes[..width]);
        self
    }
}

// ─── Writer ──────────────────────────────────────────────────────────────────

struct DfWriter<W: Write> {
    out: W,
}

impl<W: Write> DfWriter<W> {
    /// Register one message schema. All of these must precede the first data record.
    fn write_fmt(&mut self, def: &MsgDef) -> std::io::Result<()> {
        let mut p = Payload::new();
        p.u8v(def.type_id).u8v(def.record_len());
        p.text(def.name, 4);
        p.text(def.format, 16);
        p.text(def.labels, 64);
        debug_assert_eq!(p.buf.len(), FMT_DATA_LEN);
        self.out.write_all(&[HDR0, HDR1, FMT_TYPE_ID])?;
        self.out.write_all(&p.buf)
    }

    /// Write one data record. The payload must be exactly the width its schema declares — a
    /// mismatch would desynchronise every record after it, so it is an assertion, not a warning.
    fn write_rec(&mut self, def: &MsgDef, p: Payload) -> std::io::Result<()> {
        debug_assert_eq!(
            p.buf.len() + 3,
            def.record_len() as usize,
            "{} payload is {} bytes, schema declares {}",
            def.name,
            p.buf.len() + 3,
            def.record_len()
        );
        self.out.write_all(&[HDR0, HDR1, def.type_id])?;
        self.out.write_all(&p.buf)
    }

    fn msg(&mut self, time_us: u64, text: &str) -> std::io::Result<()> {
        let mut p = Payload::new();
        p.u64v(time_us).text(text, 64);
        self.write_rec(&MSG, p)
    }

    fn ev(&mut self, time_us: u64, id: u8) -> std::io::Result<()> {
        let mut p = Payload::new();
        p.u64v(time_us).u8v(id);
        self.write_rec(&EV, p)
    }
}

// ─── GPS time ────────────────────────────────────────────────────────────────

const GPS_EPOCH_UNIX_SECS: i64 = 315_964_800;
const LEAP_SECONDS: i64 = 18;

/// UTC → (GPS week, milliseconds into that week). Exact inverse of
/// `ardupilot::gps_week_ms_to_utc`, so an exported log re-imports with its original timestamps.
fn utc_to_gps_week_ms(utc: &chrono::DateTime<chrono::Utc>) -> (u16, u32) {
    let unix_ms = utc.timestamp_millis();
    let total_ms = unix_ms - GPS_EPOCH_UNIX_SECS * 1000 + LEAP_SECONDS * 1000;
    const WEEK_MS: i64 = 7 * 24 * 3600 * 1000;
    if total_ms < 0 {
        return (0, 0);
    }
    ((total_ms / WEEK_MS) as u16, (total_ms % WEEK_MS) as u32)
}

// ─── Flight-mode mapping ─────────────────────────────────────────────────────

/// True for platforms Mission Planner reads with the plane mode table (fixed wing and VTOL).
fn is_plane(platform_type: u8) -> bool {
    matches!(platform_type, 1 | 7)
}

/// Canonical mode id → ArduPilot mode number, chosen from the plane or copter table to match how
/// Mission Planner will name it. INAV ids map to their nearest equivalent; `None` when there is no
/// sensible one, in which case no MODE record is written and only the MSG name records the mode.
fn ardu_mode_num(id: &str, platform_type: u8) -> Option<u8> {
    if is_plane(platform_type) {
        // ArduPlane: 0 MANUAL, 1 CIRCLE, 2 STABILIZE, 3 TRAINING, 4 ACRO, 5 FBWA, 6 FBWB,
        // 7 CRUISE, 8 AUTOTUNE, 10 AUTO, 11 RTL, 12 LOITER, 13 TAKEOFF, 14 AVOID_ADSB,
        // 15 GUIDED, 17..23 Q-modes, 24 THERMAL, 25 LOITER_TO_QLAND.
        Some(match id {
            "manual" => 0,
            "circle" => 1,
            "stabilize" | "angle" | "horizon" => 2,
            "training" => 3,
            "acro" => 4,
            "fbwa" => 5,
            "fbwb" | "althold" => 6,
            "cruise" => 7,
            "autotune" => 8,
            "mission" | "ardu_auto" => 10,
            "rth" | "rtl" | "failsafe_rth" | "smartrtl" | "autortl" => 11,
            "loiter" | "poshold" => 12,
            "takeoff" | "launch" => 13,
            "avoid_adsb" => 14,
            "guided" | "guided_nogps" | "follow" => 15,
            "qstabilize" => 17,
            "qhover" => 18,
            "qloiter" => 19,
            "qland" => 20,
            "qrtl" => 21,
            "qautotune" => 22,
            "qacro" => 23,
            "thermal" | "soaring" => 24,
            "loiter_qland" => 25,
            // Autoland has no ArduPlane mode of its own; RTL is the closest in intent.
            "autoland" => 11,
            _ => return None,
        })
    } else {
        // ArduCopter: 0 STABILIZE, 1 ACRO, 2 ALT_HOLD, 3 AUTO, 4 GUIDED, 5 LOITER, 6 RTL,
        // 7 CIRCLE, 9 LAND, 11 DRIFT, 13 SPORT, 14 FLIP, 15 AUTOTUNE, 16 POSHOLD, 17 BRAKE,
        // 18 THROW, 19 AVOID_ADSB, 20 GUIDED_NOGPS, 21 SMART_RTL, 22 FLOWHOLD, 23 FOLLOW,
        // 24 ZIGZAG, 25 SYSTEMID, 26 AUTOROTATE, 27 AUTO_RTL.
        Some(match id {
            "stabilize" | "angle" | "horizon" | "manual" => 0,
            "acro" => 1,
            "althold" => 2,
            "mission" | "ardu_auto" => 3,
            "guided" => 4,
            "loiter" => 5,
            "rth" | "rtl" | "failsafe_rth" => 6,
            "circle" | "cruise" => 7,
            "land" | "autoland" => 9,
            "drift" => 11,
            "sport" => 13,
            "flip" => 14,
            "autotune" => 15,
            "poshold" => 16,
            "brake" => 17,
            "throw" | "launch" | "takeoff" => 18,
            "avoid_adsb" => 19,
            "guided_nogps" => 20,
            "smartrtl" => 21,
            "flowhold" => 22,
            "follow" => 23,
            "zigzag" => 24,
            "systemid" => 25,
            "autorotate" => 26,
            "autortl" => 27,
            _ => return None,
        })
    }
}

// ─── Export ──────────────────────────────────────────────────────────────────

/// Write a flight as an ArduPilot DataFlash `.bin` log.
///
/// Every recorded sample is written; nothing is GPS-filtered or simplified, since this is a log for
/// analysis rather than a path for a map. Samples without a position still contribute their
/// attitude, baro and battery records.
pub fn export_bin(
    flight: &Flight,
    track: &[TelemetryRecord],
    output_path: &Path,
) -> Result<(), String> {
    let file = std::fs::File::create(output_path)
        .map_err(|e| format!("Failed to create .bin file: {}", e))?;
    let mut w = DfWriter { out: std::io::BufWriter::new(file) };
    write_log(&mut w, flight, track).map_err(|e| format!("Failed to write .bin log: {}", e))?;
    w.out.flush().map_err(|e| format!("Failed to flush .bin log: {}", e))
}

fn write_log<W: Write>(
    w: &mut DfWriter<W>,
    flight: &Flight,
    track: &[TelemetryRecord],
) -> std::io::Result<()> {
    for def in ALL_DEFS {
        w.write_fmt(def)?;
    }

    // Provenance, so someone opening this in Mission Planner can see where it came from and that
    // it is a re-encoding rather than a log the flight controller itself wrote.
    w.msg(0, "MaxGCS DataFlash export (re-encoded from recorded telemetry)")?;
    let craft = if flight.craft_name.is_empty() { "unknown" } else { flight.craft_name.as_str() };
    w.msg(0, &format!("Craft: {} ({})", craft, flight.protocol))?;
    if !flight.fc_variant.is_empty() {
        w.msg(0, &format!("FC: {} {}", flight.fc_variant, flight.fc_version))?;
    }
    w.msg(0, &format!("Flight {} started {}", flight.id, flight.start_time.format("%Y-%m-%d %H:%M:%S UTC")))?;
    if let Some(ref loc) = flight.location_name {
        w.msg(0, &format!("Location: {}", loc))?;
    }

    let plane = flight.platform_type;
    let (gps_week, gps_ms_at_start) = utc_to_gps_week_ms(&flight.start_time);

    // The recording covers arm→disarm, so the log brackets itself with ArduPilot's arm events.
    w.ev(0, EV_ARMED)?;

    let mut last_mode: Option<String> = None;

    for r in track {
        let time_us = (r.timestamp_ms.max(0) as u64).saturating_mul(1000);

        // ── Mode change ──
        if let Some(mode) = r.mode_primary.as_deref() {
            if last_mode.as_deref() != Some(mode) {
                // The canonical name goes in an MSG regardless, because the MODE number below is
                // only ever an approximation for a non-ArduPilot flight.
                w.msg(time_us, &format!("Mode: {}", mode))?;
                if let Some(num) = ardu_mode_num(mode, plane) {
                    let mut p = Payload::new();
                    p.u64v(time_us).u8v(num).u8v(num).u8v(0);
                    w.write_rec(&MODE, p)?;
                }
                last_mode = Some(mode.to_string());
            }
        }

        // ── GPS ──
        // Only samples with a real position produce a GPS record; a fix-less sample would put a
        // null-island point on Mission Planner's map.
        if let (Some(lat), Some(lon)) = (r.lat, r.lon) {
            if lat != 0.0 || lon != 0.0 {
                let mut p = Payload::new();
                p.u64v(time_us)
                    .u8v(r.fix_type.unwrap_or(0))
                    .u32v(gps_ms_at_start.wrapping_add(r.timestamp_ms.max(0) as u32))
                    .u16v(gps_week)
                    .u8v(r.num_sat.unwrap_or(0))
                    .centi_i16(r.gps_hdop.unwrap_or(0.0))
                    .latlon(lat)
                    .latlon(lon)
                    .centi_i32(r.alt_m.or(r.baro_alt_m).unwrap_or(0.0))
                    .f32v(r.speed_ms.unwrap_or(0.0))
                    .f32v(r.heading.unwrap_or(0.0))
                    .f32v(-r.vario_ms.unwrap_or(0.0)) // ArduPilot VZ is down-positive
                    .f32v(r.yaw.unwrap_or(0.0))
                    .u8v(1); // U: this GPS instance was used for the solution
                w.write_rec(&GPS, p)?;
            }
        }

        // ── Attitude ──
        if let (Some(roll), Some(pitch)) = (r.roll, r.pitch) {
            let yaw = r.yaw.or(r.heading).unwrap_or(0.0).rem_euclid(360.0);
            let mut p = Payload::new();
            // Desired roll/pitch/yaw are the controller's targets, which recorded telemetry does
            // not carry; they repeat the achieved values so the graphs stay readable.
            p.u64v(time_us)
                .centi_i16(roll)
                .centi_i16(roll)
                .centi_i16(pitch)
                .centi_i16(pitch)
                .centi_u16(yaw)
                .centi_u16(yaw)
                .centi_u16(0.0)
                .centi_u16(0.0);
            w.write_rec(&ATT, p)?;
        }

        // ── Baro ──
        if let Some(alt) = r.baro_alt_m.or(r.nav_alt_m) {
            let mut p = Payload::new();
            p.u64v(time_us)
                .f32v(alt)
                .f32v(r.vario_ms.unwrap_or(0.0))
                .centi_i16(r.baro_temperature.unwrap_or(0.0));
            w.write_rec(&BARO, p)?;
        }

        // ── Battery ──
        if let Some(volt) = r.voltage {
            let mut p = Payload::new();
            p.u64v(time_us)
                .u8v(0)
                .f32v(volt)
                .f32v(r.current_a.unwrap_or(0.0))
                .f32v(r.mah_drawn.unwrap_or(0) as f64)
                .u8v(r.battery_percentage.unwrap_or(0));
            w.write_rec(&BAT, p)?;
        }

        // ── Airspeed ──
        if let Some(airspeed) = r.airspeed_ms {
            let mut p = Payload::new();
            p.u64v(time_us).f32v(airspeed);
            w.write_rec(&ARSP, p)?;
        }

        // ── RC input ──
        if let Some(ref json) = r.rc_data_json {
            if let Some(ch) = first_four_channels(json) {
                let mut p = Payload::new();
                p.u64v(time_us);
                for c in ch {
                    p.u16v(c);
                }
                w.write_rec(&RCIN, p)?;
            }
        }
    }

    let end_us = track
        .last()
        .map(|r| (r.timestamp_ms.max(0) as u64).saturating_mul(1000))
        .unwrap_or(0);
    w.ev(end_us, EV_DISARMED)?;
    w.msg(end_us, &format!("Export complete: {} samples", track.len()))?;
    Ok(())
}

/// First four RC channels from the stored `rc_data_json` array, if it parses and has them.
fn first_four_channels(json: &str) -> Option<[u16; 4]> {
    let v: serde_json::Value = serde_json::from_str(json).ok()?;
    let arr = v.as_array()?;
    if arr.len() < 4 {
        return None;
    }
    let mut out = [0u16; 4];
    for (i, slot) in out.iter_mut().enumerate() {
        *slot = arr[i].as_u64()?.min(u16::MAX as u64) as u16;
    }
    Some(out)
}

// ─── Tests ───────────────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use crate::flightlog::ardupilot::DataFlashScanner;
    use chrono::{TimeZone, Utc};

    fn blank_flight() -> Flight {
        let start = Utc.with_ymd_and_hms(2026, 6, 15, 9, 0, 0).unwrap();
        Flight {
            id: 0,
            start_time: start,
            end_time: None,
            duration_sec: None,
            source: String::new(),
            craft_name: String::new(),
            fc_variant: String::new(),
            fc_version: String::new(),
            board_id: String::new(),
            platform_type: 0,
            protocol: String::new(),
            start_lat: None,
            start_lon: None,
            location_name: None,
            weather_temp_c: None,
            weather_wind_ms: None,
            weather_wind_deg: None,
            weather_desc: None,
            max_alt_m: None,
            max_speed_ms: None,
            max_distance_m: None,
            total_distance_m: None,
            battery_used_mah: None,
            notes: None,
            linked_flight_id: None,
            pilot_name: None,
            pilot_id: None,
            battery_serial: None,
            utc_offset_min: None,
        }
    }

    fn at(t_ms: i64) -> TelemetryRecord {
        TelemetryRecord {
            id: 0,
            flight_id: 0,
            timestamp_ms: t_ms,
            lat: None,
            lon: None,
            alt_m: None,
            speed_ms: None,
            airspeed_ms: None,
            throttle_pct: None,
            heading: None,
            vario_ms: None,
            voltage: None,
            current_a: None,
            mah_drawn: None,
            rssi: None,
            battery_percentage: None,
            roll: None,
            pitch: None,
            yaw: None,
            fix_type: None,
            num_sat: None,
            cpu_load: None,
            link_quality: None,
            baro_alt_m: None,
            gps_hdop: None,
            gps_eph: None,
            gps_epv: None,
            active_wp_number: None,
            active_flight_mode_flags: None,
            state_flags: None,
            nav_state: None,
            nav_flags: None,
            rx_signal_received: None,
            hw_health_status: None,
            baro_temperature: None,
            wind_n_ms: None,
            wind_e_ms: None,
            wind_d_ms: None,
            rc_data_json: None,
            rc_command_json: None,
            nav_lat: None,
            nav_lon: None,
            nav_alt_m: None,
            mode_primary: None,
            mode_modifiers: None,
            link_snr: None,
            link_rssi_dbm: None,
        }
    }

    /// Encode to an in-memory buffer the same way `export_bin` writes a file.
    fn encode(flight: &Flight, track: &[TelemetryRecord]) -> Vec<u8> {
        let mut w = DfWriter { out: Vec::new() };
        write_log(&mut w, flight, track).expect("encode");
        w.out
    }

    /// Decode with the module this one mirrors, collecting every message.
    fn decode(bytes: &[u8]) -> Vec<crate::flightlog::ardupilot::ParsedMsg> {
        let mut s = DataFlashScanner::new(bytes);
        let mut out = Vec::new();
        while let Some(m) = s.next_message() {
            out.push(m);
        }
        out
    }

    fn only(msgs: &[crate::flightlog::ardupilot::ParsedMsg], name: &str)
        -> Vec<crate::flightlog::ardupilot::ParsedMsg>
    {
        msgs.iter().filter(|m| m.type_name == name).cloned().collect()
    }

    #[test]
    fn every_schema_declares_its_own_payload_width() {
        for def in ALL_DEFS {
            let w = payload_width(def.format);
            assert_eq!(def.record_len() as usize, w + 3, "{} length mismatch", def.name);
            // The reader stores record_len in a u8, so a schema must fit in one byte.
            assert!(w + 3 <= 255, "{} record too long", def.name);
            // A format char per label, so decoded field names line up with values.
            assert_eq!(
                def.format.len(),
                def.labels.split(',').count(),
                "{}: {} format chars vs {} labels",
                def.name,
                def.format.len(),
                def.labels.split(',').count()
            );
            assert!(def.name.len() <= 4, "{} name exceeds the 4-byte field", def.name);
            assert!(def.format.len() <= 16, "{} format exceeds the 16-byte field", def.name);
            assert!(def.labels.len() <= 64, "{} labels exceed the 64-byte field", def.name);
        }
    }

    #[test]
    fn type_ids_are_unique_and_avoid_the_fmt_id() {
        let mut ids: Vec<u8> = ALL_DEFS.iter().map(|d| d.type_id).collect();
        ids.sort_unstable();
        let n = ids.len();
        ids.dedup();
        assert_eq!(ids.len(), n, "duplicate type id");
        assert!(!ids.contains(&FMT_TYPE_ID), "a schema reused the FMT type id");
    }

    #[test]
    fn decoder_registers_every_schema_we_write() {
        let msgs = decode(&encode(&blank_flight(), &[at(0)]));
        let fmts = only(&msgs, "FMT");
        assert_eq!(fmts.len(), ALL_DEFS.len(), "not every FMT round-tripped");
        for def in ALL_DEFS {
            assert!(
                fmts.iter().any(|m| m.get_str("Name") == Some(def.name)),
                "{} schema missing after decode",
                def.name
            );
        }
    }

    #[test]
    fn gps_values_survive_the_round_trip() {
        let mut r = at(12_300);
        r.lat = Some(18.5204300);
        r.lon = Some(-123.8567440);   // negative, to exercise the signed 1e7 scaling
        r.alt_m = Some(104.25);
        r.speed_ms = Some(22.5);
        r.heading = Some(271.0);
        r.vario_ms = Some(1.5);
        r.fix_type = Some(3);
        r.num_sat = Some(14);
        r.gps_hdop = Some(0.94);

        let msgs = decode(&encode(&blank_flight(), &[r]));
        let gps = only(&msgs, "GPS");
        assert_eq!(gps.len(), 1);
        let g = &gps[0];
        assert_eq!(g.get_u64("TimeUS"), Some(12_300_000));
        assert!((g.get_f64("Lat").unwrap() - 18.5204300).abs() < 1e-7);
        assert!((g.get_f64("Lng").unwrap() + 123.8567440).abs() < 1e-7);
        assert!((g.get_f64("Alt").unwrap() - 104.25).abs() < 0.01);
        assert!((g.get_f64("Spd").unwrap() - 22.5).abs() < 0.001);
        assert!((g.get_f64("HDop").unwrap() - 0.94).abs() < 0.01);
        assert_eq!(g.get_f64("Status"), Some(3.0));
        assert_eq!(g.get_f64("NSats"), Some(14.0));
        // ArduPilot's VZ is down-positive, so a climb must come back negative.
        assert!((g.get_f64("VZ").unwrap() + 1.5).abs() < 0.001);
    }

    #[test]
    fn attitude_and_battery_survive_the_round_trip() {
        let mut r = at(0);
        r.roll = Some(-22.5);
        r.pitch = Some(8.25);
        r.yaw = Some(359.5);
        r.voltage = Some(16.84);
        r.current_a = Some(11.4);
        r.mah_drawn = Some(1180);
        r.battery_percentage = Some(62);

        let msgs = decode(&encode(&blank_flight(), &[r]));
        let att = &only(&msgs, "ATT")[0];
        assert!((att.get_f64("Roll").unwrap() + 22.5).abs() < 0.01);
        assert!((att.get_f64("Pitch").unwrap() - 8.25).abs() < 0.01);
        assert!((att.get_f64("Yaw").unwrap() - 359.5).abs() < 0.01);

        let bat = &only(&msgs, "BAT")[0];
        assert!((bat.get_f64("Volt").unwrap() - 16.84).abs() < 0.001);
        assert!((bat.get_f64("Curr").unwrap() - 11.4).abs() < 0.001);
        assert!((bat.get_f64("CurrTot").unwrap() - 1180.0).abs() < 0.001);
        assert_eq!(bat.get_f64("RemPct"), Some(62.0));
        assert_eq!(bat.get_f64("Instance"), Some(0.0));
    }

    #[test]
    fn gps_time_round_trips_to_the_original_utc() {
        // utc_to_gps_week_ms must invert the decoder's gps_week_ms_to_utc exactly, or an imported
        // export would land on the wrong date.
        let f = blank_flight();
        let (wk, ms) = utc_to_gps_week_ms(&f.start_time);
        let msgs = decode(&encode(&f, &[{
            let mut r = at(0);
            r.lat = Some(48.0);
            r.lon = Some(11.0);
            r.fix_type = Some(3);
            r
        }]));
        let g = &only(&msgs, "GPS")[0];
        assert_eq!(g.get_f64("GWk"), Some(wk as f64));
        assert_eq!(g.get_f64("GMS"), Some(ms as f64));
        assert!(wk > 2300, "GPS week looks wrong for a 2026 date: {}", wk);
        assert!(ms < 7 * 24 * 3600 * 1000, "GMS must stay inside the week");
    }

    #[test]
    fn fixless_samples_write_no_gps_point() {
        // A null-island sample would drop a marker off the coast of Africa on the map.
        let mut a = at(0);
        a.lat = Some(0.0);
        a.lon = Some(0.0);
        let b = at(100); // no position at all
        let msgs = decode(&encode(&blank_flight(), &[a, b]));
        assert!(only(&msgs, "GPS").is_empty());
    }

    #[test]
    fn mode_changes_emit_one_record_each_with_the_name_preserved() {
        let mut f = blank_flight();
        f.platform_type = 1; // airplane → ArduPlane mode table
        let mk = |t, m: &str| {
            let mut r = at(t);
            r.mode_primary = Some(m.into());
            r
        };
        let track = [mk(0, "angle"), mk(100, "angle"), mk(200, "rth"), mk(300, "mission")];
        let msgs = decode(&encode(&f, &track));

        let modes = only(&msgs, "MODE");
        assert_eq!(modes.len(), 3, "one record per change, not per sample");
        assert_eq!(modes[0].get_f64("Mode"), Some(2.0));  // STABILIZE
        assert_eq!(modes[1].get_f64("Mode"), Some(11.0)); // RTL
        assert_eq!(modes[2].get_f64("Mode"), Some(10.0)); // AUTO

        // The canonical name is preserved in MSG, since the number is an approximation.
        let names: Vec<String> = only(&msgs, "MSG")
            .iter()
            .filter_map(|m| m.get_str("Message").map(str::to_string))
            .filter(|s| s.starts_with("Mode: "))
            .collect();
        assert_eq!(names, ["Mode: angle", "Mode: rth", "Mode: mission"]);
    }

    #[test]
    fn copter_and_plane_use_different_mode_tables() {
        assert_eq!(ardu_mode_num("mission", 1), Some(10)); // plane AUTO
        assert_eq!(ardu_mode_num("mission", 0), Some(3));  // copter AUTO
        assert_eq!(ardu_mode_num("poshold", 1), Some(12)); // plane LOITER
        assert_eq!(ardu_mode_num("poshold", 0), Some(16)); // copter POSHOLD
        assert_eq!(ardu_mode_num("qloiter", 7), Some(19)); // VTOL reads the plane table
        assert_eq!(ardu_mode_num("some_future_mode", 0), None);
    }

    #[test]
    fn an_unmappable_mode_writes_no_mode_record_but_still_names_itself() {
        let mut r = at(0);
        r.mode_primary = Some("some_future_mode".into());
        let msgs = decode(&encode(&blank_flight(), &[r]));
        assert!(only(&msgs, "MODE").is_empty(), "guessed a mode number it could not know");
        assert!(only(&msgs, "MSG")
            .iter()
            .any(|m| m.get_str("Message") == Some("Mode: some_future_mode")));
    }

    #[test]
    fn log_is_bracketed_by_arm_and_disarm_events() {
        let msgs = decode(&encode(&blank_flight(), &[at(0), at(5_000)]));
        let evs = only(&msgs, "EV");
        assert_eq!(evs.len(), 2);
        assert_eq!(evs[0].get_f64("Id"), Some(EV_ARMED as f64));
        assert_eq!(evs[1].get_f64("Id"), Some(EV_DISARMED as f64));
        assert_eq!(evs[1].get_u64("TimeUS"), Some(5_000_000));
    }

    #[test]
    fn provenance_says_the_log_was_re_encoded() {
        let msgs = decode(&encode(&blank_flight(), &[at(0)]));
        let texts: Vec<String> = only(&msgs, "MSG")
            .iter()
            .filter_map(|m| m.get_str("Message").map(str::to_string))
            .collect();
        assert!(
            texts.iter().any(|t| t.contains("re-encoded")),
            "a reader must be able to tell this is not an FC-written log: {:?}",
            texts
        );
    }

    #[test]
    fn extreme_values_clamp_instead_of_wrapping() {
        // A corrupt telemetry sample must not be able to desynchronise the record stream: the
        // debug_asserts in write_rec would fire if any encoder wrote the wrong width.
        let mut r = at(0);
        r.roll = Some(1.0e9);
        r.pitch = Some(-1.0e9);
        r.yaw = Some(-5.0);
        r.lat = Some(91.0);
        r.lon = Some(-181.0);
        r.alt_m = Some(f64::INFINITY);
        r.speed_ms = Some(f64::NAN);
        r.voltage = Some(1.0e12);
        r.fix_type = Some(3);
        let bytes = encode(&blank_flight(), &[r]);
        let msgs = decode(&bytes);
        assert_eq!(only(&msgs, "GPS").len(), 1);
        assert_eq!(only(&msgs, "ATT").len(), 1);
        // NaN speed is written as 0 rather than a bit pattern the reader would show as garbage.
        assert_eq!(only(&msgs, "GPS")[0].get_f64("Spd"), Some(0.0));
    }

    #[test]
    fn a_long_flight_encodes_every_sample_in_time_order() {
        let track: Vec<_> = (0..500)
            .map(|i| {
                let mut r = at(i * 100);
                r.lat = Some(48.0 + i as f64 * 1e-5);
                r.lon = Some(11.0 + i as f64 * 1e-5);
                r.fix_type = Some(3);
                r.roll = Some(5.0);
                r.pitch = Some(2.0);
                r
            })
            .collect();
        let msgs = decode(&encode(&blank_flight(), &track));
        let gps = only(&msgs, "GPS");
        assert_eq!(gps.len(), 500);
        let times: Vec<u64> = gps.iter().map(|g| g.get_u64("TimeUS").unwrap()).collect();
        assert!(times.windows(2).all(|w| w[0] < w[1]), "records left time order");
        assert_eq!(times[499], 499 * 100_000);
    }

    #[test]
    fn rc_channels_are_written_only_when_the_json_parses() {
        assert_eq!(first_four_channels("[1500,1501,1502,1503,1504]"), Some([1500, 1501, 1502, 1503]));
        assert_eq!(first_four_channels("[1500,1501]"), None);
        assert_eq!(first_four_channels("not json"), None);
        assert_eq!(first_four_channels("{}"), None);
    }
}
