// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

//! Exporting saved recordings to a shareable file.
//!
//! Recordings are written as Matroska because the live feed ends by killing ffmpeg outright (see
//! `recording::new_recording_path`), and `.mkv` survives that. `.mkv` is also the one container
//! most players and upload forms refuse, so this module re-containers it as `.mp4` — optionally
//! running a stronger filter pass than the live encode can afford, since export is not realtime.
//!
//! The source recording is never modified or removed. Export only ever writes a new file.
//!
//! No neural-network filtering here: the bundled ffmpeg lists the DNN filters (`sr`,
//! `dnn_processing`) but is built without a DNN backend, so they fail at runtime. These presets are
//! classical filters, which is also what the live `RECORD_ENHANCE_FILTERS` chain uses.

use std::io::{BufRead, BufReader};
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Mutex, OnceLock};

use serde::Deserialize;

use super::ffmpeg;

/// How much work to do on the way out.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "lowercase")]
pub enum ExportPreset {
    /// Re-container only: identical pixels, no re-encode, near-instant.
    Original,
    /// A second cleanup pass — temporal denoise, sharpen, mild contrast/saturation.
    Cleaned,
    /// Cleanup plus a 2× Lanczos upscale.
    Upscale,
}

impl ExportPreset {
    /// Filter chain for this preset, or `None` to stream-copy.
    ///
    /// The recording already carries the live chain's mild spatial denoise, so the extra pass here
    /// is `atadenoise` — purely temporal, averaging each pixel across neighbouring frames. It
    /// removes the per-frame sensor noise the live filter had to leave behind without softening
    /// detail the way a second spatial denoise would.
    fn filters(self) -> Option<&'static str> {
        match self {
            ExportPreset::Original => None,
            ExportPreset::Cleaned => Some(
                "atadenoise,unsharp=5:5:0.8:5:5:0.0,eq=contrast=1.05:saturation=1.06",
            ),
            // Denoise before scaling, so the upscaler magnifies detail rather than grain. The
            // sharpen is gentler than in Cleaned because Lanczos already adds local contrast.
            ExportPreset::Upscale => Some(
                "atadenoise,scale=iw*2:ih*2:flags=lanczos,\
                 unsharp=5:5:0.6:5:5:0.0,eq=contrast=1.05:saturation=1.06",
            ),
        }
    }

    /// x264 speed/quality trade-off. The upscale preset pushes four times the pixels, so it gets a
    /// faster preset to keep the export from taking longer than the flight did.
    fn encode_args(self) -> &'static [&'static str] {
        match self {
            ExportPreset::Original => &[],
            ExportPreset::Cleaned => &["-preset", "medium", "-crf", "20"],
            ExportPreset::Upscale => &["-preset", "fast", "-crf", "19"],
        }
    }
}

// ── Cancellation ─────────────────────────────────────────────────────────────
//
// One export at a time, which matches the UI: the Recordings panel disables the button while a job
// runs. The child is parked in a static so the cancel command can reach it without the export
// having to hold a lock for its whole (possibly minutes-long) duration.

static ACTIVE: OnceLock<Mutex<Option<Child>>> = OnceLock::new();
static CANCELLED: AtomicBool = AtomicBool::new(false);

fn active() -> &'static Mutex<Option<Child>> {
    ACTIVE.get_or_init(|| Mutex::new(None))
}

/// Kill a running export. The partial output file is removed by `run`, which owns the path.
/// Returns false when nothing was running.
pub fn cancel() -> bool {
    CANCELLED.store(true, Ordering::SeqCst);
    let mut guard = match active().lock() {
        Ok(g) => g,
        Err(e) => e.into_inner(),
    };
    match guard.as_mut() {
        Some(child) => {
            let _ = child.kill();
            true
        }
        None => false,
    }
}

pub fn is_running() -> bool {
    active().lock().map(|g| g.is_some()).unwrap_or(false)
}

// ── Duration probe ───────────────────────────────────────────────────────────

/// Total duration in seconds, read from ffmpeg's own banner.
///
/// `ffmpeg -i <file>` with no output writes the stream summary to stderr and exits non-zero ("At
/// least one output file must be specified") — that non-zero exit is expected, we only want the
/// `Duration:` line. ffprobe would be the natural tool but is not bundled.
fn probe_duration_secs(ff: &Path, source: &Path) -> Option<f64> {
    let out = Command::new(ff)
        .args(["-hide_banner", "-i"])
        .arg(source)
        .stdin(Stdio::null())
        .output()
        .ok()?;
    let text = String::from_utf8_lossy(&out.stderr);
    let after = text.split("Duration:").nth(1)?;
    let stamp = after.split(',').next()?.trim();
    parse_timestamp(stamp)
}

/// `HH:MM:SS.ss` → seconds. Returns None for ffmpeg's "N/A" on a stream with no known duration.
fn parse_timestamp(s: &str) -> Option<f64> {
    let mut parts = s.split(':');
    let h: f64 = parts.next()?.trim().parse().ok()?;
    let m: f64 = parts.next()?.trim().parse().ok()?;
    let sec: f64 = parts.next()?.trim().parse().ok()?;
    Some(h * 3600.0 + m * 60.0 + sec)
}

// ── Export ───────────────────────────────────────────────────────────────────

/// Suggested output filename for a recording: same stem, `.mp4`, with the preset noted when the
/// pixels changed — so an exported copy is not mistaken for the untouched original.
pub fn suggested_name(source: &Path, preset: ExportPreset) -> String {
    let stem = source.file_stem().and_then(|s| s.to_str()).unwrap_or("recording");
    match preset {
        ExportPreset::Original => format!("{stem}.mp4"),
        ExportPreset::Cleaned => format!("{stem}_cleaned.mp4"),
        ExportPreset::Upscale => format!("{stem}_cleaned_2x.mp4"),
    }
}

/// Transcode (or re-container) `source` to `output`. `report(pct, msg)` is called as ffmpeg
/// progresses; `pct` reaches 100 only on success.
///
/// `source` is opened read-only and left exactly as it was, whatever the outcome.
pub fn run<F>(
    source: &Path,
    output: &Path,
    preset: ExportPreset,
    mut report: F,
) -> Result<(), String>
where
    F: FnMut(u8, &str),
{
    if is_running() {
        return Err("An export is already running.".into());
    }
    if !source.is_file() {
        return Err(format!("Recording not found: {}", source.display()));
    }
    // Writing over the source would truncate the only copy of the footage before ffmpeg had read it.
    if same_file(source, output) {
        return Err("Choose a different file — export cannot overwrite the recording itself.".into());
    }
    let ff = ffmpeg::find_ffmpeg()
        .ok_or_else(|| "ffmpeg was not found. Install it from the Video panel first.".to_string())?;

    CANCELLED.store(false, Ordering::SeqCst);
    report(0, "Reading recording…");
    let total = probe_duration_secs(&ff, source);

    let mut cmd = Command::new(&ff);
    cmd.args(["-hide_banner", "-nostdin", "-y", "-i"])
        .arg(source)
        // Video plus audio when the source happens to have a track; `?` makes the audio optional
        // rather than failing the whole export on a video-only recording.
        .args(["-map", "0:v:0", "-map", "0:a?"]);

    match preset.filters() {
        Some(filters) => {
            cmd.args(["-vf", filters])
                .args(["-c:v", "libx264"])
                .args(preset.encode_args())
                // yuv420p for players that reject the capture's native pixel format.
                .args(["-pix_fmt", "yuv420p", "-c:a", "copy"]);
        }
        None => {
            cmd.args(["-c", "copy"]);
        }
    }

    cmd.args(["-movflags", "+faststart"])
        // Machine-readable progress on stdout; the human-readable stats would fight with it.
        .args(["-progress", "pipe:1", "-nostats"])
        .arg(output)
        .stdin(Stdio::null())
        .stdout(Stdio::piped())
        .stderr(Stdio::null());

    let mut child = cmd.spawn().map_err(|e| format!("Could not start ffmpeg: {e}"))?;
    let stdout = child
        .stdout
        .take()
        .ok_or_else(|| "ffmpeg produced no progress stream.".to_string())?;

    // Park the child so `cancel` can reach it while we sit reading progress.
    if let Ok(mut g) = active().lock() {
        *g = Some(child);
    }

    report(1, "Exporting…");
    for line in BufReader::new(stdout).lines().map_while(Result::ok) {
        let Some((key, value)) = line.split_once('=') else { continue };
        if key != "out_time_us" {
            continue;
        }
        let Ok(us) = value.trim().parse::<i64>() else { continue };
        // Without a known duration there is nothing to divide by, so the bar holds at 1% and the
        // elapsed position carries the message instead.
        match total {
            Some(t) if t > 0.0 => {
                let pct = ((us as f64 / 1e6) / t * 100.0).clamp(1.0, 99.0) as u8;
                report(pct, "Exporting…");
            }
            _ => report(1, &format!("Exporting… {:.0}s", us as f64 / 1e6)),
        }
    }

    let status = {
        let mut guard = match active().lock() {
            Ok(g) => g,
            Err(e) => e.into_inner(),
        };
        match guard.take() {
            Some(mut c) => c.wait().map_err(|e| format!("ffmpeg did not exit cleanly: {e}"))?,
            None => return Err("Export state was lost.".into()),
        }
    };

    if CANCELLED.swap(false, Ordering::SeqCst) {
        // A killed ffmpeg leaves a half-written file that is not a playable video.
        let _ = std::fs::remove_file(output);
        return Err("Export cancelled.".into());
    }

    if !status.success() {
        let _ = std::fs::remove_file(output);
        return Err(format!(
            "ffmpeg failed ({}). The recording itself is untouched.",
            status.code().map_or_else(|| "killed".to_string(), |c| c.to_string())
        ));
    }

    report(100, "Export complete");
    Ok(())
}

/// Whether two paths name the same existing file. Compared by canonical path, so a relative path or
/// a symlink to the source is still caught; an output that does not exist yet trivially differs.
fn same_file(a: &Path, b: &Path) -> bool {
    match (a.canonicalize(), b.canonicalize()) {
        (Ok(x), Ok(y)) => x == y,
        _ => false,
    }
}
