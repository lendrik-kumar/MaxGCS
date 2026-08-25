// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

//! Video subsystem (backend). v1 was frontend-only (webcam/USB via getUserMedia); this adds the
//! "native backend source": live RTSP via the go2rtc engine (RTSP→WebRTC), with ffmpeg as go2rtc's
//! fallback RTSP reader for sources its native client can't handle. See docs/active/RTSP_VIDEO.md.
//!
//! Linux V4L2 capture devices (HDMI dongles, etc.) that aren't exposed via getUserMedia are
//! enumerated and ingested through the same go2rtc/ffmpeg pipeline.

pub mod ffmpeg;
pub mod go2rtc;
pub mod native;
/// V4L2 device enumeration is Linux-only (reads `/sys/class/video4linux`); `native` uses it there.
#[cfg(target_os = "linux")]
pub mod v4l2;
pub mod mjpeg_server;

pub use go2rtc::Go2Rtc;
pub use mjpeg_server::MjpegServer;
