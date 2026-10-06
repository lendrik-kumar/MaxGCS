// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Flight Recording & Logbook Module
// Records telemetry data during flights (arm→disarm) into a SQLite database.
// Supports optional raw text logging, reverse geocoding, and weather metadata.
// Recorded flights export as geometry (track_export: KMZ/KML/GPX/CSV), as readable logs with
// an event timeline (text_export: TXT/MD/JSON), or as an ArduPilot DataFlash .bin for Mission
// Planner (dataflash_export).

pub mod db;
pub mod ardupilot;
pub mod dataflash_export;
pub mod blackbox;
pub mod decoder;
pub mod exchange;
pub mod geocode;
pub mod msp_raw_logger;
pub mod raw_import;
pub mod recorder;
pub mod text_export;
pub mod timezone;
pub mod tlog_logger;
pub mod track_export;
pub mod types;
pub mod weather;
