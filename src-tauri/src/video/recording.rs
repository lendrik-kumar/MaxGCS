// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

//! Enhanced native-capture recordings. `mjpeg_server.rs` writes the optional second (filtered,
//! encoded) ffmpeg output here when recording is toggled on; this module is where that file lives
//! and how the frontend's Recordings panel lists them back.

use serde::Serialize;
use std::path::PathBuf;
use std::time::UNIX_EPOCH;

/// One saved recording, as listed for the frontend's Recordings panel.
#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "camelCase")]
pub struct RecordingInfo {
    pub name: String,
    pub path: String,
    pub size_bytes: u64,
    /// Last-modified time, ms since epoch (the recording's end time — ffmpeg finalizes the file's
    /// mtime on exit, not on creation).
    pub modified_ms: i64,
}

/// Where recordings are saved: `Documents/MaxGCS/Recordings` — mirrors
/// `flightlog::db::resolve_raw_log_dir`'s reasoning: user-facing output belongs in Documents (easy to
/// find, survives an AppData wipe), not the AppData folder the DB/logs use. Not user-configurable yet
/// (the master toggle lives in Settings; a custom path can follow the same pattern later if wanted).
///
/// Named "MaxGCS" rather than "KiteGC" (unlike the flightlog DB/logs, which still use the pre-rename
/// name app-wide) since this is a brand-new folder with no existing user data to migrate.
pub fn recordings_dir() -> PathBuf {
    let base = dirs::document_dir()
        .or_else(dirs::home_dir)
        .unwrap_or_else(|| PathBuf::from("."));
    base.join("MaxGCS").join("Recordings")
}

/// A fresh, timestamped output path inside `recordings_dir()`. Creates the directory if missing.
///
/// `.mkv`, not `.mp4`: the recording is ended by killing ffmpeg outright (see `MjpegServer::stop`),
/// and Matroska tolerates that — no trailing global index that a hard kill can leave unwritten,
/// unlike MP4's `moov` atom.
pub fn new_recording_path() -> Result<PathBuf, String> {
    let dir = recordings_dir();
    std::fs::create_dir_all(&dir).map_err(|e| format!("could not create the recordings folder: {e}"))?;
    let stamp = chrono::Local::now().format("%Y-%m-%d_%H-%M-%S");
    Ok(dir.join(format!("recording_{stamp}.mkv")))
}

/// List saved recordings, newest first. An absent folder (nothing recorded yet) is not an error.
pub fn list() -> Result<Vec<RecordingInfo>, String> {
    let dir = recordings_dir();
    if !dir.is_dir() {
        return Ok(Vec::new());
    }
    let mut out = Vec::new();
    for entry in std::fs::read_dir(&dir).map_err(|e| format!("could not read the recordings folder: {e}"))? {
        let Ok(entry) = entry else { continue };
        let path = entry.path();
        if path.extension().and_then(|e| e.to_str()) != Some("mkv") {
            continue;
        }
        let Ok(meta) = entry.metadata() else { continue };
        let modified_ms = meta
            .modified()
            .ok()
            .and_then(|t| t.duration_since(UNIX_EPOCH).ok())
            .map(|d| d.as_millis() as i64)
            .unwrap_or(0);
        out.push(RecordingInfo {
            name: entry.file_name().to_string_lossy().into_owned(),
            path: path.to_string_lossy().into_owned(),
            size_bytes: meta.len(),
            modified_ms,
        });
    }
    out.sort_by(|a, b| b.modified_ms.cmp(&a.modified_ms));
    Ok(out)
}
