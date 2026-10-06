// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Human-readable flight log export — plain text, Markdown and JSON.
//
// `track_export.rs` produces *geometry* for mapping tools (KMZ/KML/GPX) plus a bare column dump
// (CSV). This module produces logs meant to be read: a metadata header, a derived event timeline
// (recording start/stop, mode changes, GPS fix transitions, waypoint progress, RC link loss,
// battery milestones, flight peaks) and the full telemetry table at recorded resolution.
//
// All three formats are built from the same `build_timeline` / `Computed` pair, so the numbers and
// events never disagree between them. Unlike the geometry exporters these never drop a sample:
// no GPS filtering, no Douglas-Peucker simplification, and a GPS-less flight still logs fine.
//
// Times appear as a `T+` offset from the recording start next to flight-local wall-clock time
// (`Flight::utc_offset_min`, ADR-048), so a timeline lines up with the logbook clock.

use std::fmt::Write as _;
use std::path::Path;

use chrono::{DateTime, Duration, FixedOffset, Utc};
use serde::Serialize;

use super::track_export::{best_alt_relative, haversine_m};
use super::types::{Flight, TelemetryRecord};

// ── Event timeline ──────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "lowercase")]
pub enum Severity {
    Info,
    Warn,
    Error,
}

impl Severity {
    /// Fixed-width marker so the text timeline stays column-aligned.
    fn tag(self) -> &'static str {
        match self {
            Severity::Info => "      ",
            Severity::Warn => "[WARN]",
            Severity::Error => "[FAIL]",
        }
    }
}

#[derive(Debug, Clone, Serialize)]
pub struct TimelineEvent {
    /// Milliseconds since recording start (same epoch as `TelemetryRecord::timestamp_ms`).
    pub t_ms: i64,
    /// Machine-readable kind, for tooling that consumes the JSON log.
    pub kind: &'static str,
    pub severity: Severity,
    pub message: String,
}

fn ev(t_ms: i64, kind: &'static str, severity: Severity, message: String) -> TimelineEvent {
    TimelineEvent { t_ms, kind, severity, message }
}

/// Battery state-of-charge milestones, descending. Each is reported once, the first time the pack
/// drops to or below it — a brief sag past a threshold does not re-arm it.
const BATTERY_MILESTONES: [u8; 7] = [75, 50, 30, 20, 15, 10, 5];

/// Satellite-count hysteresis: warn below `SAT_LOW`, clear at `SAT_OK` or above.
const SAT_LOW: u8 = 6;
const SAT_OK: u8 = 7;

/// Derive the event timeline by walking the samples and reporting every state transition.
///
/// Nothing here is read from a dedicated events table (there is none) — the recorder stores mode,
/// fix, waypoint and link state on every sample, so the transitions are recoverable after the fact.
/// This also means the timeline works for imported blackbox/raw logs, not just live recordings.
pub fn build_timeline(flight: &Flight, track: &[TelemetryRecord]) -> Vec<TimelineEvent> {
    let mut out: Vec<TimelineEvent> = Vec::new();
    let Some(first) = track.first() else {
        return out;
    };
    let last = track.last().unwrap();

    let craft = if flight.craft_name.is_empty() { "unknown craft" } else { flight.craft_name.as_str() };
    out.push(ev(
        first.timestamp_ms,
        "recording_start",
        Severity::Info,
        format!("Recording started — {} ({})", craft, flight.protocol),
    ));

    // Transition state, all `None` until the first sample that carries the field.
    let mut prev_mode: Option<String> = None;
    let mut prev_mods: Option<String> = None;
    let mut prev_fix: Option<u8> = None;
    let mut prev_wp: Option<i32> = None;
    let mut prev_rx: Option<u8> = None;
    let mut prev_nav_state: Option<i32> = None;
    let mut prev_hw_health: Option<i64> = None;
    let mut sats_low = false;
    let mut next_milestone = 0usize;

    for r in track {
        let t = r.timestamp_ms;

        // ── Flight mode ──
        if let Some(mode) = r.mode_primary.as_deref() {
            if prev_mode.as_deref() != Some(mode) {
                let sev = if is_emergency_mode(mode) { Severity::Warn } else { Severity::Info };
                out.push(ev(t, "mode", sev, format!("Flight mode → {}", mode_label(mode))));
                prev_mode = Some(mode.to_string());
            }
        }
        let mods = r.mode_modifiers.as_deref().unwrap_or("");
        if prev_mods.as_deref() != Some(mods) {
            // Skip the no-modifiers → no-modifiers case on the very first sample.
            if !(prev_mods.is_none() && mods.is_empty()) {
                let msg = if mods.is_empty() {
                    "Mode modifiers cleared".to_string()
                } else {
                    format!("Mode modifiers → {}", labels_csv(mods))
                };
                out.push(ev(t, "mode_modifiers", Severity::Info, msg));
            }
            prev_mods = Some(mods.to_string());
        }

        // ── GPS fix ──
        if let Some(fix) = r.fix_type {
            if prev_fix != Some(fix) {
                let had_fix = prev_fix.is_some_and(|p| p >= 2);
                let has_fix = fix >= 2;
                let sats = r.num_sat.unwrap_or(0);
                if has_fix {
                    let hdop = r.gps_hdop.map_or(String::new(), |h| format!(", HDOP {:.1}", h));
                    out.push(ev(
                        t,
                        "gps_fix",
                        Severity::Info,
                        format!("GPS {} ({} sats{})", fix_label(fix), sats, hdop),
                    ));
                } else if had_fix {
                    out.push(ev(
                        t,
                        "gps_fix",
                        Severity::Warn,
                        format!("GPS fix lost ({} sats)", sats),
                    ));
                }
                prev_fix = Some(fix);
            }

            // Satellite count, with hysteresis so a sample flickering around the threshold does
            // not produce a run of alternating events.
            if fix >= 2 {
                if let Some(sats) = r.num_sat {
                    if !sats_low && sats < SAT_LOW {
                        sats_low = true;
                        out.push(ev(
                            t,
                            "gps_sats",
                            Severity::Warn,
                            format!("GPS satellite count low ({} sats)", sats),
                        ));
                    } else if sats_low && sats >= SAT_OK {
                        sats_low = false;
                        out.push(ev(
                            t,
                            "gps_sats",
                            Severity::Info,
                            format!("GPS satellite count recovered ({} sats)", sats),
                        ));
                    }
                }
            }
        }

        // ── Mission progress ──
        if let Some(wp) = r.active_wp_number {
            if prev_wp != Some(wp) {
                if prev_wp.is_some() || wp > 0 {
                    let msg = if wp > 0 {
                        format!("Waypoint #{} active", wp)
                    } else {
                        "Mission waypoint cleared".to_string()
                    };
                    out.push(ev(t, "waypoint", Severity::Info, msg));
                }
                prev_wp = Some(wp);
            }
        }

        // ── RC link ──
        if let Some(rx) = r.rx_signal_received {
            if prev_rx.is_some() && prev_rx != Some(rx) {
                if rx == 0 {
                    out.push(ev(t, "rc_link", Severity::Error, "RC link lost".to_string()));
                } else {
                    out.push(ev(t, "rc_link", Severity::Info, "RC link restored".to_string()));
                }
            }
            prev_rx = Some(rx);
        }

        // ── Navigation FSM ──
        if let Some(ns) = r.nav_state {
            if prev_nav_state != Some(ns) {
                if prev_nav_state.is_some() {
                    out.push(ev(t, "nav_state", Severity::Info, format!("Nav state → {}", ns)));
                }
                prev_nav_state = Some(ns);
            }
        }

        // ── Hardware health ──
        if let Some(hw) = r.hw_health_status {
            if prev_hw_health != Some(hw) {
                if prev_hw_health.is_some() {
                    let sev = if hw != 0 { Severity::Warn } else { Severity::Info };
                    out.push(ev(
                        t,
                        "hw_health",
                        sev,
                        format!("Hardware health flags → 0x{:X}", hw),
                    ));
                }
                prev_hw_health = Some(hw);
            }
        }

        // ── Battery state of charge ──
        if let Some(pct) = r.battery_percentage {
            while next_milestone < BATTERY_MILESTONES.len()
                && pct <= BATTERY_MILESTONES[next_milestone]
            {
                let m = BATTERY_MILESTONES[next_milestone];
                let sev = if m <= 10 {
                    Severity::Error
                } else if m <= 20 {
                    Severity::Warn
                } else {
                    Severity::Info
                };
                let volt = r.voltage.map_or(String::new(), |v| format!(", {:.2} V", v));
                out.push(ev(t, "battery", sev, format!("Battery at {}%{}", m, volt)));
                next_milestone += 1;
            }
        }
    }

    // ── Flight peaks ──
    let c = Computed::compute(flight, track);
    if let (Some(alt), Some(t)) = (c.max_alt_m, c.max_alt_t_ms) {
        out.push(ev(t, "peak_altitude", Severity::Info, format!("Peak altitude {:.1} m", alt)));
    }
    if let (Some(spd), Some(t)) = (c.max_speed_ms, c.max_speed_t_ms) {
        out.push(ev(
            t,
            "peak_speed",
            Severity::Info,
            format!("Peak ground speed {:.1} m/s ({:.1} km/h)", spd, spd * 3.6),
        ));
    }
    if let (Some(dist), Some(t)) = (c.max_dist_m, c.max_dist_t_ms) {
        out.push(ev(
            t,
            "peak_distance",
            Severity::Info,
            format!("Furthest from home {:.0} m", dist),
        ));
    }

    let dur = fmt_duration_ms(last.timestamp_ms - first.timestamp_ms);
    out.push(ev(
        last.timestamp_ms,
        "recording_stop",
        Severity::Info,
        format!("Recording stopped — {} of data, {} samples", dur, track.len()),
    ));

    // Peaks and the stop marker were appended out of order; a stable sort keeps same-timestamp
    // events in the order they were discovered.
    out.sort_by_key(|e| e.t_ms);
    out
}

/// Modes that mean something went wrong rather than something was commanded.
fn is_emergency_mode(id: &str) -> bool {
    matches!(id, "failsafe" | "failsafe_rth" | "avoid_adsb" | "autorotate" | "px4_termination")
}

fn fix_label(fix: u8) -> &'static str {
    match fix {
        0 => "no fix",
        1 => "dead reckoning",
        2 => "2D fix",
        3 => "3D fix",
        4 => "3D fix + DGPS",
        5 => "RTK float",
        6 => "RTK fixed",
        _ => "fix",
    }
}

/// Label a comma-separated list of canonical mode ids (the `mode_modifiers` column shape).
fn labels_csv(ids: &str) -> String {
    ids.split(',')
        .map(str::trim)
        .filter(|s| !s.is_empty())
        .map(mode_label)
        .collect::<Vec<_>>()
        .join(", ")
}

/// Canonical mode id → display label. Mirrors the frontend `flightModeRegistry.ts` so a log reads
/// the same as the mode badge did in flight; unknown ids fall through as-is.
fn mode_label(id: &str) -> &str {
    match id {
        // INAV primaries
        "failsafe_rth" => "Failsafe RTH",
        "failsafe" => "Failsafe",
        "mission" => "Mission",
        "rth" => "RTH",
        "launch" => "Launch",
        "poshold" => "PosHold",
        "cruise" => "Cruise",
        "angle" => "Angle",
        "horizon" => "Horizon",
        "manual" => "Manual",
        "acro" => "Acro",
        // INAV modifiers
        "althold" => "AltHold",
        "headinghold" => "Heading",
        "headfree" => "HeadFree",
        "soaring" => "Soaring",
        "autotune" => "AutoTune",
        "flaperon" => "Flaperon",
        "autoland" => "Autoland",
        // ArduPilot
        "ardu_auto" => "Auto",
        "rtl" => "RTL",
        "guided" => "Guided",
        "guided_nogps" => "Guided NoGPS",
        "loiter" => "Loiter",
        "circle" => "Circle",
        "land" => "Land",
        "stabilize" => "Stabilize",
        "training" => "Training",
        "fbwa" => "FBW-A",
        "fbwb" => "FBW-B",
        "takeoff" => "Takeoff",
        "avoid_adsb" => "Avoid ADSB",
        "drift" => "Drift",
        "sport" => "Sport",
        "flip" => "Flip",
        "brake" => "Brake",
        "throw" => "Throw",
        "smartrtl" => "SmartRTL",
        "flowhold" => "FlowHold",
        "follow" => "Follow",
        "zigzag" => "ZigZag",
        "systemid" => "SystemID",
        "autorotate" => "Autorotate",
        "autortl" => "AutoRTL",
        "qstabilize" => "QStabilize",
        "qhover" => "QHover",
        "qloiter" => "QLoiter",
        "qland" => "QLand",
        "qrtl" => "QRTL",
        "qautotune" => "QAutoTune",
        "qacro" => "QAcro",
        "thermal" => "Thermal",
        "loiter_qland" => "Loiter QLand",
        // PX4
        "px4_altitude" => "Altitude",
        "px4_position" => "Position",
        "px4_orbit" => "Orbit",
        "px4_offboard" => "Offboard",
        "px4_stabilized" => "Stabilized",
        "px4_rattitude" => "Rattitude",
        "px4_return" => "Return",
        "px4_hold" => "Hold",
        "px4_ready" => "Ready",
        "px4_follow_me" => "Follow Me",
        "px4_precland" => "Precision Land",
        "px4_vtol_takeoff" => "VTOL Takeoff",
        "px4_termination" => "Termination",
        other => other,
    }
}

fn platform_label(t: u8) -> &'static str {
    match t {
        0 => "Multirotor",
        1 => "Airplane",
        2 => "Helicopter",
        3 => "Tricopter",
        4 => "Rover",
        5 => "Boat",
        6 => "Other",
        7 => "VTOL",
        255 => "Generic (no FC identity)",
        _ => "Unknown",
    }
}

// ── Derived statistics ──────────────────────────────────────────────

/// Statistics recomputed from the samples, with the timestamp of each peak so the timeline can
/// mark *when* it happened. Independent of the summary columns on `Flight`, which are written by
/// the recorder and may be absent on imported logs.
#[derive(Debug, Clone, Serialize)]
pub struct Computed {
    pub samples: usize,
    pub span_ms: i64,
    pub avg_rate_hz: Option<f64>,
    pub gps_samples: usize,
    pub home: Option<(f64, f64)>,
    pub max_alt_m: Option<f64>,
    pub max_alt_t_ms: Option<i64>,
    pub max_speed_ms: Option<f64>,
    pub max_speed_t_ms: Option<i64>,
    pub max_dist_m: Option<f64>,
    pub max_dist_t_ms: Option<i64>,
    pub min_voltage: Option<f64>,
    pub max_current_a: Option<f64>,
    pub max_mah_drawn: Option<u32>,
}

impl Computed {
    pub fn compute(flight: &Flight, track: &[TelemetryRecord]) -> Self {
        let samples = track.len();
        let span_ms = match (track.first(), track.last()) {
            (Some(f), Some(l)) => l.timestamp_ms - f.timestamp_ms,
            _ => 0,
        };
        let avg_rate_hz = if samples > 1 && span_ms > 0 {
            Some((samples - 1) as f64 / (span_ms as f64 / 1000.0))
        } else {
            None
        };

        // Home: the recorder's stored start position, else the first sample with a real fix.
        let home = match (flight.start_lat, flight.start_lon) {
            (Some(la), Some(lo)) => Some((la, lo)),
            _ => track
                .iter()
                .find(|r| has_gps(r))
                .and_then(|r| Some((r.lat?, r.lon?))),
        };

        let mut c = Self {
            samples,
            span_ms,
            avg_rate_hz,
            gps_samples: track.iter().filter(|r| has_gps(r)).count(),
            home,
            max_alt_m: None,
            max_alt_t_ms: None,
            max_speed_ms: None,
            max_speed_t_ms: None,
            max_dist_m: None,
            max_dist_t_ms: None,
            min_voltage: None,
            max_current_a: None,
            max_mah_drawn: None,
        };

        for r in track {
            let alt = best_alt_relative(r);
            if c.max_alt_m.is_none_or(|m| alt > m) {
                c.max_alt_m = Some(alt);
                c.max_alt_t_ms = Some(r.timestamp_ms);
            }
            if let Some(s) = r.speed_ms {
                if c.max_speed_ms.is_none_or(|m| s > m) {
                    c.max_speed_ms = Some(s);
                    c.max_speed_t_ms = Some(r.timestamp_ms);
                }
            }
            if let (Some((hla, hlo)), true) = (home, has_gps(r)) {
                let d = haversine_m(hla, hlo, r.lat.unwrap(), r.lon.unwrap());
                if c.max_dist_m.is_none_or(|m| d > m) {
                    c.max_dist_m = Some(d);
                    c.max_dist_t_ms = Some(r.timestamp_ms);
                }
            }
            if let Some(v) = r.voltage {
                // Ignore the 0 V samples a link emits before the battery reading arrives.
                if v > 0.0 && c.min_voltage.is_none_or(|m| v < m) {
                    c.min_voltage = Some(v);
                }
            }
            if let Some(a) = r.current_a {
                if c.max_current_a.is_none_or(|m| a > m) {
                    c.max_current_a = Some(a);
                }
            }
            if let Some(m) = r.mah_drawn {
                if c.max_mah_drawn.is_none_or(|p| m > p) {
                    c.max_mah_drawn = Some(m);
                }
            }
        }

        if c.max_alt_m.is_none() {
            c.max_alt_t_ms = None;
        }
        c
    }
}

/// A sample with a usable position: both coordinates present and not the null island that a
/// fix-less FC reports.
fn has_gps(r: &TelemetryRecord) -> bool {
    matches!((r.lat, r.lon), (Some(la), Some(lo)) if la != 0.0 || lo != 0.0)
}

// ── Time formatting ─────────────────────────────────────────────────

/// Flight-local offset (ADR-048). Falls back to UTC for rows without GPS-derived timezone data.
fn local_offset(flight: &Flight) -> FixedOffset {
    let utc = FixedOffset::east_opt(0).expect("UTC is a valid fixed offset");
    flight
        .utc_offset_min
        .and_then(|m| FixedOffset::east_opt(m.saturating_mul(60)))
        .unwrap_or(utc)
}

/// `T+HH:MM:SS.mmm`, fixed width so timeline and table columns line up.
fn fmt_offset_ms(ms: i64) -> String {
    let neg = ms < 0;
    let ms = ms.abs();
    let h = ms / 3_600_000;
    let m = (ms % 3_600_000) / 60_000;
    let s = (ms % 60_000) / 1000;
    let frac = ms % 1000;
    format!("{}{:02}:{:02}:{:02}.{:03}", if neg { "-" } else { "+" }, h, m, s, frac)
}

/// Human duration: `1h 02m 13s`, or `18m 51s` under an hour.
fn fmt_duration_ms(ms: i64) -> String {
    let total = ms.max(0) / 1000;
    let h = total / 3600;
    let m = (total % 3600) / 60;
    let s = total % 60;
    if h > 0 {
        format!("{}h {:02}m {:02}s", h, m, s)
    } else {
        format!("{}m {:02}s", m, s)
    }
}

fn fmt_duration_sec(sec: i64) -> String {
    fmt_duration_ms(sec.saturating_mul(1000))
}

/// Absolute wall-clock time of a sample, in flight-local time.
fn sample_local(flight_start: &DateTime<Utc>, t_ms: i64, off: &FixedOffset) -> DateTime<FixedOffset> {
    (*flight_start + Duration::milliseconds(t_ms)).with_timezone(off)
}

// ── Header ──────────────────────────────────────────────────────────

/// Label/value pairs for the metadata block, shared by the text and Markdown renderers so both
/// report exactly the same facts. Absent fields are omitted rather than printed empty.
fn header_fields(flight: &Flight, c: &Computed) -> Vec<(&'static str, String)> {
    let off = local_offset(flight);
    let mut f: Vec<(&'static str, String)> = Vec::new();

    f.push(("Flight ID", flight.id.to_string()));
    f.push((
        "Craft",
        if flight.craft_name.is_empty() { "—".to_string() } else { flight.craft_name.clone() },
    ));
    f.push(("Platform", platform_label(flight.platform_type).to_string()));

    let fc = [flight.fc_variant.as_str(), flight.fc_version.as_str()]
        .iter()
        .filter(|s| !s.is_empty())
        .cloned()
        .collect::<Vec<_>>()
        .join(" ");
    if !fc.is_empty() {
        let board = if flight.board_id.is_empty() {
            String::new()
        } else {
            format!("  (board {})", flight.board_id)
        };
        f.push(("Autopilot", format!("{}{}", fc, board)));
    }
    f.push(("Protocol", flight.protocol.clone()));
    f.push(("Log source", flight.source.clone()));

    if let Some(ref p) = flight.pilot_name {
        let id = flight.pilot_id.as_deref().filter(|s| !s.is_empty()).map_or(String::new(), |i| format!("  (ID {})", i));
        f.push(("Pilot", format!("{}{}", p, id)));
    }
    if let Some(ref b) = flight.battery_serial {
        f.push(("Battery pack", b.clone()));
    }

    f.push((
        "Start (local)",
        format!(
            "{}  UTC{}",
            flight.start_time.with_timezone(&off).format("%Y-%m-%d %H:%M:%S"),
            off
        ),
    ));
    f.push((
        "Start (UTC)",
        flight.start_time.format("%Y-%m-%dT%H:%M:%SZ").to_string(),
    ));
    if let Some(end) = flight.end_time {
        f.push((
            "End (local)",
            end.with_timezone(&off).format("%Y-%m-%d %H:%M:%S").to_string(),
        ));
    }
    if let Some(d) = flight.duration_sec {
        f.push(("Duration", fmt_duration_sec(d)));
    }

    if let Some(ref loc) = flight.location_name {
        f.push(("Location", loc.clone()));
    }
    if let Some((la, lo)) = c.home {
        f.push(("Home position", format!("{:.7}, {:.7}", la, lo)));
    }

    let mut wx: Vec<String> = Vec::new();
    if let Some(t) = flight.weather_temp_c {
        wx.push(format!("{:.1} °C", t));
    }
    if let Some(w) = flight.weather_wind_ms {
        let dir = flight.weather_wind_deg.map_or(String::new(), |d| format!(" from {}°", d));
        wx.push(format!("wind {:.1} m/s{}", w, dir));
    }
    if let Some(ref d) = flight.weather_desc {
        wx.push(d.clone());
    }
    if !wx.is_empty() {
        f.push(("Weather at start", wx.join(", ")));
    }

    // Recomputed from the samples — authoritative even when the summary columns are empty.
    if let Some(a) = c.max_alt_m {
        f.push(("Max altitude", format!("{:.1} m (relative to home)", a)));
    }
    if let Some(s) = c.max_speed_ms {
        f.push(("Max ground speed", format!("{:.1} m/s  ({:.1} km/h)", s, s * 3.6)));
    }
    if let Some(d) = c.max_dist_m {
        f.push(("Max distance", format!("{:.0} m from home", d)));
    }
    if let Some(d) = flight.total_distance_m {
        f.push(("Total distance", format!("{:.0} m", d)));
    }
    if let Some(v) = c.min_voltage {
        f.push(("Min pack voltage", format!("{:.2} V", v)));
    }
    if let Some(a) = c.max_current_a {
        f.push(("Max current", format!("{:.1} A", a)));
    }
    if let Some(m) = flight.battery_used_mah.or(c.max_mah_drawn) {
        f.push(("Battery used", format!("{} mAh", m)));
    }

    let rate = c.avg_rate_hz.map_or(String::new(), |hz| format!("  (avg {:.1} Hz)", hz));
    f.push((
        "Samples",
        format!("{} over {}{}", c.samples, fmt_duration_ms(c.span_ms), rate),
    ));
    f.push((
        "Samples with GPS",
        format!(
            "{} of {}",
            c.gps_samples,
            c.samples
        ),
    ));

    if let Some(ref n) = flight.notes {
        if !n.trim().is_empty() {
            f.push(("Notes", n.replace('\n', " / ")));
        }
    }

    f
}

// ── Telemetry table ─────────────────────────────────────────────────

/// The telemetry table's column layout, as one `{:>w}` slot per field. The header row and every
/// data row both go through this macro, so the labels can never drift out of alignment with the
/// values underneath them. Widths fit the worst realistic case (negative longitudes, four-digit
/// altitudes, -180° attitudes) so a wide value never pushes the rest of its row sideways.
macro_rules! telemetry_row {
    ($($arg:expr),+ $(,)?) => {
        format!(
            concat!(
                "{:>13}  {:>12}  {:>11}  {:>12}  {:>7}  {:>5}  {:>5}  {:>6}  {:>4}  {:>4}  ",
                "{:>6}  {:>6}  {:>4}  {:>6}  {:>6}  {:>6}  {:>3}  {:>3}  {:>3}  {:>4}  ",
                "{:>4}  {:>3}  {:>3}  {}"
            ),
            $($arg),+
        )
    };
}

fn table_header() -> String {
    telemetry_row!(
        "T+", "LOCAL TIME", "LAT", "LON", "ALT", "SPD", "AIR", "VS", "HDG", "YAW", "ROLL", "PITCH",
        "THR", "VOLT", "CURR", "MAH", "SOC", "SAT", "FIX", "HDOP", "RSSI", "LQ", "WP", "MODE"
    )
}

/// One telemetry row. Blank cells mean the field was absent from that sample — never zero-filled,
/// so a missing sensor stays distinguishable from a real zero reading.
fn table_row(r: &TelemetryRecord, flight_start: &DateTime<Utc>, off: &FixedOffset) -> String {
    telemetry_row!(
        fmt_offset_ms(r.timestamp_ms),
        // chrono's Display writes straight to the formatter and ignores width, so the timestamp
        // has to become an owned String before it can be padded into its column.
        sample_local(flight_start, r.timestamp_ms, off).format("%H:%M:%S%.3f").to_string(),
        num(r.lat, 7),
        num(r.lon, 7),
        num(Some(best_alt_relative(r)), 1),
        num(r.speed_ms, 1),
        num(r.airspeed_ms, 1),
        num(r.vario_ms, 1),
        num(r.heading, 0),
        num(r.yaw, 0),
        num(r.roll, 1),
        num(r.pitch, 1),
        num(r.throttle_pct, 0),
        num(r.voltage, 2),
        num(r.current_a, 1),
        int(r.mah_drawn),
        int(r.battery_percentage),
        int(r.num_sat),
        int(r.fix_type),
        num(r.gps_hdop, 1),
        int(r.rssi),
        int(r.link_quality),
        int(r.active_wp_number),
        mode_cell(r),
    )
}

fn mode_cell(r: &TelemetryRecord) -> String {
    let primary = r.mode_primary.as_deref().map(mode_label).unwrap_or("—").to_string();
    match r.mode_modifiers.as_deref().filter(|m| !m.is_empty()) {
        Some(m) => format!("{} + {}", primary, labels_csv(m)),
        None => primary,
    }
}

fn num(v: Option<f64>, decimals: usize) -> String {
    v.map_or(String::new(), |f| format!("{:.*}", decimals, f))
}

fn int<T: std::fmt::Display>(v: Option<T>) -> String {
    v.map_or(String::new(), |x| x.to_string())
}

// ── Plain text ──────────────────────────────────────────────────────

const RULE: &str = "────────────────────────────────────────────────────────────────────────────────";

/// Write the full human-readable log: metadata header, event timeline, telemetry table.
pub fn export_txt(
    flight: &Flight,
    track: &[TelemetryRecord],
    output_path: &Path,
) -> Result<(), String> {
    let out = build_txt(flight, track);
    std::fs::write(output_path, out).map_err(|e| format!("Failed to write text log: {}", e))
}

pub fn build_txt(flight: &Flight, track: &[TelemetryRecord]) -> String {
    let c = Computed::compute(flight, track);
    let off = local_offset(flight);
    let mut s = String::with_capacity(track.len() * 180 + 4096);

    let _ = writeln!(s, "{}", RULE);
    let _ = writeln!(s, " MaxGCS Flight Log");
    let _ = writeln!(s, "{}", RULE);
    let _ = writeln!(
        s,
        " Exported {}   ·   all times in flight-local time (UTC{}) unless marked",
        Utc::now().with_timezone(&off).format("%Y-%m-%d %H:%M:%S"),
        off
    );
    let _ = writeln!(s);

    // ── Metadata ──
    let fields = header_fields(flight, &c);
    let width = fields.iter().map(|(k, _)| k.len()).max().unwrap_or(0);
    for (k, v) in &fields {
        let _ = writeln!(s, " {:<width$} : {}", k, v, width = width);
    }
    let _ = writeln!(s);

    if track.is_empty() {
        let _ = writeln!(s, "{}", RULE);
        let _ = writeln!(s, " No telemetry samples were recorded for this flight.");
        let _ = writeln!(s, "{}", RULE);
        return s;
    }

    // ── Timeline ──
    let timeline = build_timeline(flight, track);
    let _ = writeln!(s, "{}", RULE);
    let _ = writeln!(s, " EVENT TIMELINE   ({} events)", timeline.len());
    let _ = writeln!(s, "{}", RULE);
    for e in &timeline {
        let _ = writeln!(
            s,
            " {:>13}  {:>12}  {}  {}",
            fmt_offset_ms(e.t_ms),
            sample_local(&flight.start_time, e.t_ms, &off).format("%H:%M:%S%.3f").to_string(),
            e.severity.tag(),
            e.message
        );
    }
    let _ = writeln!(s);

    // ── Telemetry ──
    let _ = writeln!(s, "{}", RULE);
    let _ = writeln!(s, " TELEMETRY   ({} samples, full recorded resolution)", track.len());
    let _ = writeln!(
        s,
        " Units: ALT/m rel. home · SPD/AIR/VS m/s · HDG/YAW/ROLL/PITCH deg · THR/SOC % · VOLT V · CURR A"
    );
    let _ = writeln!(s, " A blank cell means the sample carried no value for that field.");
    let _ = writeln!(s, "{}", RULE);
    let _ = writeln!(s, "{}", table_header());
    for r in track {
        let _ = writeln!(s, "{}", table_row(r, &flight.start_time, &off));
    }
    let _ = writeln!(s, "{}", RULE);
    let _ = writeln!(s, " End of log — flight {}, {} samples", flight.id, track.len());
    let _ = writeln!(s, "{}", RULE);

    s
}

// ── Markdown ────────────────────────────────────────────────────────

/// Same content as the text log, structured for a Markdown reader: metadata and timeline as
/// tables, the wide telemetry grid in a fenced block so it keeps its alignment.
pub fn export_md(
    flight: &Flight,
    track: &[TelemetryRecord],
    output_path: &Path,
) -> Result<(), String> {
    let out = build_md(flight, track);
    std::fs::write(output_path, out).map_err(|e| format!("Failed to write Markdown log: {}", e))
}

pub fn build_md(flight: &Flight, track: &[TelemetryRecord]) -> String {
    let c = Computed::compute(flight, track);
    let off = local_offset(flight);
    let mut s = String::with_capacity(track.len() * 180 + 4096);

    let craft = if flight.craft_name.is_empty() { "Unknown craft" } else { flight.craft_name.as_str() };
    let _ = writeln!(
        s,
        "# Flight log — {} · {}",
        md_escape(craft),
        flight.start_time.with_timezone(&off).format("%Y-%m-%d %H:%M")
    );
    let _ = writeln!(s);
    let _ = writeln!(
        s,
        "*Exported by MaxGCS on {}. All times are flight-local (UTC{}) unless marked otherwise.*",
        Utc::now().with_timezone(&off).format("%Y-%m-%d %H:%M:%S"),
        off
    );
    let _ = writeln!(s);

    let _ = writeln!(s, "## Flight details");
    let _ = writeln!(s);
    let _ = writeln!(s, "| Field | Value |");
    let _ = writeln!(s, "| --- | --- |");
    for (k, v) in header_fields(flight, &c) {
        let _ = writeln!(s, "| {} | {} |", k, md_escape(&v));
    }
    let _ = writeln!(s);

    if track.is_empty() {
        let _ = writeln!(s, "## Telemetry");
        let _ = writeln!(s);
        let _ = writeln!(s, "No telemetry samples were recorded for this flight.");
        return s;
    }

    let timeline = build_timeline(flight, track);
    let _ = writeln!(s, "## Event timeline");
    let _ = writeln!(s);
    let _ = writeln!(s, "{} events.", timeline.len());
    let _ = writeln!(s);
    let _ = writeln!(s, "| T+ | Local time | Level | Event |");
    let _ = writeln!(s, "| --- | --- | --- | --- |");
    for e in &timeline {
        let level = match e.severity {
            Severity::Info => "info",
            Severity::Warn => "**warn**",
            Severity::Error => "**FAIL**",
        };
        let _ = writeln!(
            s,
            "| `{}` | {} | {} | {} |",
            fmt_offset_ms(e.t_ms),
            sample_local(&flight.start_time, e.t_ms, &off).format("%H:%M:%S%.3f"),
            level,
            md_escape(&e.message)
        );
    }
    let _ = writeln!(s);

    let _ = writeln!(s, "## Telemetry");
    let _ = writeln!(s);
    let _ = writeln!(
        s,
        "{} samples at full recorded resolution. Units: ALT/m relative to home · SPD/AIR/VS m/s · \
         HDG/YAW/ROLL/PITCH deg · THR/SOC % · VOLT V · CURR A. A blank cell means the sample \
         carried no value for that field.",
        track.len()
    );
    let _ = writeln!(s);
    let _ = writeln!(s, "```text");
    let _ = writeln!(s, "{}", table_header());
    for r in track {
        let _ = writeln!(s, "{}", table_row(r, &flight.start_time, &off));
    }
    let _ = writeln!(s, "```");

    s
}

/// Escape the characters that would break a Markdown table cell.
fn md_escape(s: &str) -> String {
    s.replace('\\', "\\\\").replace('|', "\\|").replace('\n', " ")
}

// ── JSON ────────────────────────────────────────────────────────────

#[derive(Serialize)]
struct JsonEvent<'a> {
    t_ms: i64,
    t_offset: String,
    time_utc: String,
    time_local: String,
    kind: &'a str,
    severity: Severity,
    message: &'a str,
}

#[derive(Serialize)]
struct JsonSample<'a> {
    t_offset: String,
    time_utc: String,
    time_local: String,
    #[serde(flatten)]
    record: &'a TelemetryRecord,
}

#[derive(Serialize)]
struct JsonLog<'a> {
    format: &'static str,
    format_version: u32,
    exported_at: String,
    /// Flight-local UTC offset in minutes (east-positive); `time_local` fields use it.
    local_utc_offset_min: i32,
    flight: &'a Flight,
    statistics: Computed,
    timeline: Vec<JsonEvent<'a>>,
    samples: Vec<JsonSample<'a>>,
}

/// Write the structured log: the same header, timeline and samples as the text log, losslessly.
/// Every `TelemetryRecord` field is included, so this is the format to parse rather than scrape.
pub fn export_json(
    flight: &Flight,
    track: &[TelemetryRecord],
    output_path: &Path,
) -> Result<(), String> {
    let out = build_json(flight, track)?;
    std::fs::write(output_path, out).map_err(|e| format!("Failed to write JSON log: {}", e))
}

pub fn build_json(flight: &Flight, track: &[TelemetryRecord]) -> Result<String, String> {
    let c = Computed::compute(flight, track);
    let off = local_offset(flight);
    let timeline = build_timeline(flight, track);

    let abs = |t_ms: i64| {
        let utc = flight.start_time + Duration::milliseconds(t_ms);
        (
            utc.format("%Y-%m-%dT%H:%M:%S%.3fZ").to_string(),
            utc.with_timezone(&off).format("%Y-%m-%dT%H:%M:%S%.3f%:z").to_string(),
        )
    };

    let events: Vec<JsonEvent> = timeline
        .iter()
        .map(|e| {
            let (utc, local) = abs(e.t_ms);
            JsonEvent {
                t_ms: e.t_ms,
                t_offset: fmt_offset_ms(e.t_ms),
                time_utc: utc,
                time_local: local,
                kind: e.kind,
                severity: e.severity,
                message: &e.message,
            }
        })
        .collect();

    let samples: Vec<JsonSample> = track
        .iter()
        .map(|r| {
            let (utc, local) = abs(r.timestamp_ms);
            JsonSample {
                t_offset: fmt_offset_ms(r.timestamp_ms),
                time_utc: utc,
                time_local: local,
                record: r,
            }
        })
        .collect();

    let log = JsonLog {
        format: "maxgcs-flight-log",
        format_version: 1,
        exported_at: Utc::now().format("%Y-%m-%dT%H:%M:%SZ").to_string(),
        local_utc_offset_min: off.local_minus_utc() / 60,
        flight,
        statistics: c,
        timeline: events,
        samples,
    };

    serde_json::to_string_pretty(&log).map_err(|e| format!("Failed to serialize JSON log: {}", e))
}

// ── Tests ───────────────────────────────────────────────────────────

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::TimeZone;

    /// A flight with every field at its empty value, so each test sets only what it exercises.
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

    /// A sample at `t_ms` with every telemetry field absent.
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

    fn kinds(events: &[TimelineEvent]) -> Vec<&str> {
        events.iter().map(|e| e.kind).collect()
    }

    #[test]
    fn timeline_is_empty_without_samples() {
        assert!(build_timeline(&blank_flight(), &[]).is_empty());
    }

    #[test]
    fn timeline_brackets_the_recording() {
        let t = build_timeline(&blank_flight(), &[at(0), at(5_000)]);
        assert_eq!(t.first().unwrap().kind, "recording_start");
        assert_eq!(t.last().unwrap().kind, "recording_stop");
    }

    #[test]
    fn timeline_is_ordered_by_time() {
        let mut a = at(0);
        a.lat = Some(48.0);
        a.lon = Some(11.0);
        a.speed_ms = Some(1.0);
        let mut b = at(10_000);
        b.lat = Some(48.01);
        b.lon = Some(11.01);
        b.speed_ms = Some(30.0);
        let t = build_timeline(&blank_flight(), &[a, b]);
        assert!(t.windows(2).all(|w| w[0].t_ms <= w[1].t_ms), "{:?}", kinds(&t));
    }

    #[test]
    fn mode_change_is_reported_once_per_transition() {
        let mut a = at(0);
        a.mode_primary = Some("angle".into());
        let mut b = at(1_000);
        b.mode_primary = Some("angle".into());
        let mut c = at(2_000);
        c.mode_primary = Some("rth".into());
        let t = build_timeline(&blank_flight(), &[a, b, c]);
        let modes: Vec<_> = t.iter().filter(|e| e.kind == "mode").collect();
        assert_eq!(modes.len(), 2, "one event per change, not per sample");
        assert!(modes[0].message.contains("Angle"));
        assert!(modes[1].message.contains("RTH"));
    }

    #[test]
    fn failsafe_mode_is_flagged_as_a_warning() {
        let mut a = at(0);
        a.mode_primary = Some("failsafe_rth".into());
        let t = build_timeline(&blank_flight(), &[a]);
        let m = t.iter().find(|e| e.kind == "mode").expect("mode event");
        assert_eq!(m.severity, Severity::Warn);
        assert!(m.message.contains("Failsafe RTH"));
    }

    #[test]
    fn gps_fix_gain_and_loss_both_report() {
        let mut a = at(0);
        a.fix_type = Some(0);
        let mut b = at(1_000);
        b.fix_type = Some(3);
        b.num_sat = Some(11);
        let mut c = at(2_000);
        c.fix_type = Some(1);
        let t = build_timeline(&blank_flight(), &[a, b, c]);
        let fixes: Vec<_> = t.iter().filter(|e| e.kind == "gps_fix").collect();
        assert_eq!(fixes.len(), 2);
        assert!(fixes[0].message.contains("3D fix") && fixes[0].message.contains("11 sats"));
        assert_eq!(fixes[1].severity, Severity::Warn);
        assert!(fixes[1].message.contains("lost"));
    }

    #[test]
    fn low_satellite_count_warns_once_until_it_recovers() {
        let sats = [10u8, 5, 4, 5, 9, 4];
        let track: Vec<_> = sats
            .iter()
            .enumerate()
            .map(|(i, &s)| {
                let mut r = at(i as i64 * 1000);
                r.fix_type = Some(3);
                r.num_sat = Some(s);
                r
            })
            .collect();
        let t = build_timeline(&blank_flight(), &track);
        let ev: Vec<_> = t.iter().filter(|e| e.kind == "gps_sats").collect();
        // low (5) → recovered (9) → low (4): the 4 and 5 in between must not re-arm the warning.
        assert_eq!(ev.len(), 3, "{:?}", ev.iter().map(|e| &e.message).collect::<Vec<_>>());
        assert_eq!(ev[0].severity, Severity::Warn);
        assert_eq!(ev[1].severity, Severity::Info);
        assert_eq!(ev[2].severity, Severity::Warn);
    }

    #[test]
    fn battery_milestones_fire_once_and_never_rearm() {
        let pct = [100u8, 80, 49, 60, 28, 9];
        let track: Vec<_> = pct
            .iter()
            .enumerate()
            .map(|(i, &p)| {
                let mut r = at(i as i64 * 1000);
                r.battery_percentage = Some(p);
                r
            })
            .collect();
        let t = build_timeline(&blank_flight(), &track);
        let msgs: Vec<_> = t
            .iter()
            .filter(|e| e.kind == "battery")
            .map(|e| e.message.clone())
            .collect();
        // 49 % crosses both 75 and 50 in one step; the recovery to 60 % must not re-report
        // either; 9 % walks the rest down to 10 % and stops there (5 % was never reached).
        let crossed: Vec<&str> = msgs
            .iter()
            .map(|m| m.trim_start_matches("Battery at ").trim_end_matches('%'))
            .collect();
        assert_eq!(crossed, ["75", "50", "30", "20", "15", "10"], "{:?}", msgs);
        let worst = t.iter().filter(|e| e.kind == "battery").last().unwrap();
        assert_eq!(worst.severity, Severity::Error);
    }

    #[test]
    fn rc_link_loss_is_an_error_and_restore_is_info() {
        let mut a = at(0);
        a.rx_signal_received = Some(1);
        let mut b = at(1_000);
        b.rx_signal_received = Some(0);
        let mut c = at(2_000);
        c.rx_signal_received = Some(1);
        let t = build_timeline(&blank_flight(), &[a, b, c]);
        let ev: Vec<_> = t.iter().filter(|e| e.kind == "rc_link").collect();
        assert_eq!(ev.len(), 2);
        assert_eq!(ev[0].severity, Severity::Error);
        assert_eq!(ev[1].severity, Severity::Info);
    }

    #[test]
    fn peaks_are_computed_from_samples_not_summary_columns() {
        let mut f = blank_flight();
        f.start_lat = Some(48.0);
        f.start_lon = Some(11.0);
        f.max_alt_m = None; // summary absent, as on an imported log

        let mut a = at(0);
        a.lat = Some(48.0);
        a.lon = Some(11.0);
        a.baro_alt_m = Some(0.0);
        a.speed_ms = Some(0.0);
        let mut b = at(10_000);
        b.lat = Some(48.0);
        b.lon = Some(11.0);
        b.baro_alt_m = Some(120.0);
        b.speed_ms = Some(22.0);

        let c = Computed::compute(&f, &[a.clone(), b.clone()]);
        assert_eq!(c.max_alt_m, Some(120.0));
        assert_eq!(c.max_alt_t_ms, Some(10_000));
        assert_eq!(c.max_speed_ms, Some(22.0));
        assert_eq!(c.samples, 2);
        assert_eq!(c.gps_samples, 2);
        assert_eq!(c.span_ms, 10_000);
        assert_eq!(c.avg_rate_hz, Some(0.1));
    }

    #[test]
    fn null_island_does_not_count_as_a_fix() {
        let mut r = at(0);
        r.lat = Some(0.0);
        r.lon = Some(0.0);
        let c = Computed::compute(&blank_flight(), &[r]);
        assert_eq!(c.gps_samples, 0);
        assert_eq!(c.home, None);
    }

    #[test]
    fn table_header_and_rows_share_one_column_layout() {
        let f = blank_flight();
        let off = local_offset(&f);
        let mut r = at(3_661_123);
        // Worst realistic case for every width: a negative longitude, a four-digit altitude,
        // -180° attitudes, a 12S pack voltage and a six-digit mAh total.
        r.lat = Some(-48.1234567);
        r.lon = Some(-123.1234567);
        r.baro_alt_m = Some(-1234.5);
        r.roll = Some(-180.0);
        r.pitch = Some(-180.0);
        r.heading = Some(359.0);
        r.voltage = Some(50.44);
        r.current_a = Some(123.4);
        r.mah_drawn = Some(123_456);
        r.rssi = Some(1023);
        r.battery_percentage = Some(100);
        r.mode_primary = Some("angle".into());

        // The layout holds only if nothing overflowed its slot, which shows up as the two-space
        // separator still sitting at every column boundary — in the header and the data row alike.
        let widths = [13, 12, 11, 12, 7, 5, 5, 6, 4, 4, 6, 6, 4, 6, 6, 6, 3, 3, 3, 4, 4, 3, 3];
        for line in [table_header(), table_row(&r, &f.start_time, &off)] {
            let b = line.as_bytes();
            let mut pos = 0usize;
            for (i, w) in widths.iter().enumerate() {
                pos += w;
                assert!(pos + 2 <= b.len(), "line ends inside column {}:\n{}", i, line);
                assert_eq!(
                    &b[pos..pos + 2],
                    b"  ",
                    "column {} overflowed its {} chars:\n{}",
                    i,
                    w,
                    line
                );
                pos += 2;
            }
        }
    }

    #[test]
    fn txt_log_is_written_for_a_flight_with_no_gps() {
        let f = blank_flight();
        let mut r = at(0);
        r.voltage = Some(16.8);
        let out = build_txt(&f, &[r, at(1_000)]);
        assert!(out.contains("MaxGCS Flight Log"));
        assert!(out.contains("EVENT TIMELINE"));
        assert!(out.contains("TELEMETRY"));
        assert!(out.contains("Recording started"));
        // The geometry exporters reject this flight; the readable log must not.
        assert!(!out.contains("No valid GPS"));
    }

    #[test]
    fn txt_log_handles_a_flight_with_no_samples() {
        let out = build_txt(&blank_flight(), &[]);
        assert!(out.contains("No telemetry samples"));
        assert!(!out.contains("EVENT TIMELINE"));
    }

    #[test]
    fn json_log_round_trips_as_valid_json() {
        let mut f = blank_flight();
        f.craft_name = "Test | Wing".into();
        f.utc_offset_min = Some(330); // UTC+05:30
        let mut r = at(1_500);
        r.lat = Some(18.5204300);
        r.lon = Some(73.8567440);
        r.mode_primary = Some("mission".into());
        r.active_wp_number = Some(3);

        let s = build_json(&f, &[r]).expect("json builds");
        let v: serde_json::Value = serde_json::from_str(&s).expect("valid json");
        assert_eq!(v["format"], "maxgcs-flight-log");
        assert_eq!(v["local_utc_offset_min"], 330);
        assert_eq!(v["samples"].as_array().unwrap().len(), 1);
        // Flattened record fields sit next to the added absolute timestamps.
        assert_eq!(v["samples"][0]["active_wp_number"], 3);
        assert!(v["samples"][0]["time_local"].as_str().unwrap().ends_with("+05:30"));
        assert!(!v["timeline"].as_array().unwrap().is_empty());
        assert_eq!(v["statistics"]["samples"], 1);
    }

    #[test]
    fn markdown_escapes_pipes_so_tables_survive() {
        let mut f = blank_flight();
        f.craft_name = "Wing | A".into();
        f.notes = Some("line one\nline two | piped".into());
        let out = build_md(&f, &[at(0)]);
        assert!(out.contains("Wing \\| A"), "craft name pipe not escaped");
        // Notes fold their newlines into " / " before escaping, so the cell stays on one row.
        assert!(
            out.contains("line one / line two \\| piped"),
            "notes cell not folded and escaped"
        );
        for line in out.lines().filter(|l| l.starts_with('|')) {
            assert_eq!(line.lines().count(), 1, "table cell spans rows: {}", line);
        }
        assert!(out.contains("## Event timeline"));
    }

    #[test]
    fn offsets_and_durations_format_predictably() {
        assert_eq!(fmt_offset_ms(0), "+00:00:00.000");
        assert_eq!(fmt_offset_ms(3_661_123), "+01:01:01.123");
        assert_eq!(fmt_offset_ms(-1_500), "-00:00:01.500");
        assert_eq!(fmt_duration_ms(1_131_000), "18m 51s");
        assert_eq!(fmt_duration_ms(3_733_000), "1h 02m 13s");
    }

    #[test]
    fn local_offset_falls_back_to_utc() {
        let mut f = blank_flight();
        assert_eq!(local_offset(&f).local_minus_utc(), 0);
        f.utc_offset_min = Some(-480);
        assert_eq!(local_offset(&f).local_minus_utc(), -480 * 60);
    }

    #[test]
    fn unknown_mode_ids_pass_through_unchanged() {
        assert_eq!(mode_label("angle"), "Angle");
        assert_eq!(mode_label("some_future_mode"), "some_future_mode");
        assert_eq!(labels_csv("althold,headfree"), "AltHold, HeadFree");
        assert_eq!(labels_csv(""), "");
    }
}

