// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

//! Fixed 1920×1080 output canvas. Every frame that leaves the backend video pipeline — the live
//! MJPEG feed and the recorded leg — is placed at the **centre of a black 1920×1080 canvas**:
//!
//! * the picture keeps its own resolution and aspect ratio — no stretching, no cropping, no upscaling;
//! * whatever the picture does not cover is black padding;
//! * the one exception is a source *larger* than the canvas, which cannot be placed 1:1 without
//!   cropping, so it is scaled **down** (aspect preserved) until it fits. It is never scaled up.
//!
//! The work is one ffmpeg filter chain (`scale` + `pad` + `setsar`), so it adds no process, no pipe and
//! no per-frame copy through this program. `placement` mirrors the chain's arithmetic exactly and is
//! what the tests (and any caller that needs the rectangle) use.

/// Canvas width in pixels.
pub const CANVAS_W: u32 = 1920;
/// Canvas height in pixels.
pub const CANVAS_H: u32 = 1080;

/// Where a `src_w`×`src_h` picture lands on the canvas: `(width, height, x, y)` of the placed picture.
///
/// Mirrors the filter chain below: a picture that already fits is placed as it is; a larger one is
/// scaled down by the single factor that makes it fit (aspect preserved, never upscaled); the result
/// is centred, with the offset rounded down to an even pixel (chroma-subsampling alignment, which is
/// what ffmpeg's `pad` does) so a 1-pixel difference at most from true centre.
pub fn placement(src_w: u32, src_h: u32) -> (u32, u32, u32, u32) {
    if src_w == 0 || src_h == 0 {
        return (0, 0, 0, 0);
    }
    let (w, h) = if src_w <= CANVAS_W && src_h <= CANVAS_H {
        (src_w, src_h)
    } else {
        // Same rule as `scale=…:force_original_aspect_ratio=decrease`: the binding axis is the one
        // with the larger overshoot; the other is derived from the aspect ratio.
        let by_w = (CANVAS_W as u64 * src_h as u64 / src_w as u64) as u32;
        let by_h = (CANVAS_H as u64 * src_w as u64 / src_h as u64) as u32;
        if (src_w as u64 * CANVAS_H as u64) >= (src_h as u64 * CANVAS_W as u64) {
            (CANVAS_W, by_w.max(1))
        } else {
            (by_h.max(1), CANVAS_H)
        }
    };
    let x = ((CANVAS_W - w) / 2) & !1;
    let y = ((CANVAS_H - h) / 2) & !1;
    (w, h, x, y)
}

/// The ffmpeg filter chain that puts the input at the centre of the black canvas, as a single
/// comma-joined chain (usable after `-vf`, or as the body of a `-filter_complex` label).
///
/// * `scale` only ever *reduces* (`min(canvas, input)` on each axis + `force_original_aspect_ratio=
///   decrease`), so an input that fits passes through untouched — same pixels, same size;
/// * `pad` centres it on the canvas with black;
/// * `setsar=1` states that the result has square pixels, so a player does not stretch the placed
///   picture a second time (analog SD sources carry a non-square sample aspect ratio).
///
/// The quotes keep ffmpeg's filtergraph parser from treating the commas inside `min(…)` as filter
/// separators; no shell is involved (arguments are passed straight to the process).
pub fn filter() -> String {
    format!(
        "scale=w='min({w},iw)':h='min({h},ih)':force_original_aspect_ratio=decrease,\
         pad={w}:{h}:(ow-iw)/2:(oh-ih)/2:color=black,setsar=1",
        w = CANVAS_W,
        h = CANVAS_H,
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn small_pictures_are_centred_untouched() {
        // Analog SD, 720p and a 1080p picture: placed at their own size, never upscaled.
        assert_eq!(placement(720, 480), (720, 480, 600, 300));
        assert_eq!(placement(1280, 720), (1280, 720, 320, 180));
        assert_eq!(placement(640, 480), (640, 480, 640, 300));
        assert_eq!(placement(1920, 1080), (1920, 1080, 0, 0));
    }

    #[test]
    fn narrow_and_wide_pictures_pad_on_the_free_axis_only() {
        // Full-height 4:3 → pillarbox; full-width 2.39:1 → letterbox.
        assert_eq!(placement(1440, 1080), (1440, 1080, 240, 0));
        assert_eq!(placement(1920, 800), (1920, 800, 0, 140));
    }

    #[test]
    fn oversized_pictures_shrink_to_fit_without_changing_aspect() {
        // 4K 16:9 → exactly the canvas.
        assert_eq!(placement(3840, 2160), (1920, 1080, 0, 0));
        // 4K DCI (17:9) is wider than 16:9 → bound by width, letterboxed.
        let (w, h, x, y) = placement(4096, 2160);
        assert_eq!((w, x), (1920, 0));
        assert_eq!(h, 1012);
        assert_eq!(y, 34);
        // A tall 1080×2160 picture is bound by height, pillarboxed.
        let (w, h, x, y) = placement(1080, 2160);
        assert_eq!((w, h, y), (540, 1080, 0));
        assert_eq!(x, 690);
    }

    #[test]
    fn placement_never_leaves_the_canvas_and_never_upscales() {
        for &(sw, sh) in &[(1, 1), (2, 2), (321, 241), (719, 479), (1919, 1079), (1921, 1081), (7680, 4320), (100, 5000)] {
            let (w, h, x, y) = placement(sw, sh);
            assert!(w >= 1 && h >= 1, "{sw}x{sh} collapsed");
            assert!(x + w <= CANVAS_W && y + h <= CANVAS_H, "{sw}x{sh} overflows the canvas");
            if sw <= CANVAS_W && sh <= CANVAS_H {
                assert_eq!((w, h), (sw, sh), "{sw}x{sh} must not be rescaled");
            }
            // Centred to within the even-pixel alignment (1 px) plus the odd-size remainder (3 px at most).
            assert!((x as i64 * 2 + w as i64 - CANVAS_W as i64).abs() <= 3, "{sw}x{sh} off-centre in x");
            assert!((y as i64 * 2 + h as i64 - CANVAS_H as i64).abs() <= 3, "{sw}x{sh} off-centre in y");
        }
    }

    #[test]
    fn degenerate_input_is_not_placed() {
        assert_eq!(placement(0, 480), (0, 0, 0, 0));
        assert_eq!(placement(720, 0), (0, 0, 0, 0));
    }

    #[test]
    fn filter_names_the_fixed_canvas_and_only_reduces() {
        let f = filter();
        assert!(f.contains("pad=1920:1080:(ow-iw)/2:(oh-ih)/2:color=black"), "{f}");
        assert!(f.contains("min(1920,iw)") && f.contains("min(1080,ih)"), "{f}");
        assert!(f.contains("force_original_aspect_ratio=decrease"), "{f}");
        assert!(f.ends_with("setsar=1"), "{f}");
        // One chain: no stray whitespace/newlines from the line continuation.
        assert!(!f.contains(char::is_whitespace), "{f}");
    }

    /// Runs the real ffmpeg when one is installed (skipped silently otherwise — CI machines and fresh
    /// checkouts usually have none): renders a 640×360 test picture through `filter()` and checks the
    /// raw output is exactly 1920×1080 with the picture at the centre and black everywhere else.
    #[test]
    fn real_ffmpeg_output_is_a_centred_1080p_canvas() {
        let Some(ff) = crate::video::ffmpeg::find_ffmpeg() else {
            eprintln!("ffmpeg not found — skipping the end-to-end canvas check");
            return;
        };
        for &(sw, sh) in &[(640u32, 360u32), (720, 480), (1280, 720), (2560, 1440), (321, 241)] {
            let mut cmd = std::process::Command::new(&ff);
            cmd.args([
                "-hide_banner", "-loglevel", "error", "-f", "lavfi", "-i",
                &format!("color=c=white:size={sw}x{sh}:rate=1:duration=1"),
                "-frames:v", "1", "-vf", &filter(), "-pix_fmt", "gray", "-f", "rawvideo", "-",
            ]);
            let out = cmd.output().expect("run ffmpeg");
            assert!(out.status.success(), "ffmpeg failed: {}", String::from_utf8_lossy(&out.stderr));
            let px = out.stdout;
            assert_eq!(px.len(), (CANVAS_W * CANVAS_H) as usize, "{sw}x{sh}: output is not 1920x1080");
            let (w, h, x, y) = placement(sw, sh);
            let at = |cx: u32, cy: u32| px[(cy * CANVAS_W + cx) as usize];
            // Inside the placed rectangle (one pixel in from each edge): the white test picture.
            assert!(at(x + 1, y + 1) > 200, "{sw}x{sh}: picture missing at its top-left");
            assert!(at(x + w - 2, y + h - 2) > 200, "{sw}x{sh}: picture missing at its bottom-right");
            // The canvas corners and, if there is a margin, just outside the picture: black.
            for &(cx, cy) in &[(0, 0), (CANVAS_W - 1, 0), (0, CANVAS_H - 1), (CANVAS_W - 1, CANVAS_H - 1)] {
                if cx < x || cx >= x + w || cy < y || cy >= y + h {
                    assert!(at(cx, cy) < 30, "{sw}x{sh}: corner ({cx},{cy}) is not black");
                }
            }
            if x >= 2 {
                assert!(at(x - 2, y + h / 2) < 30, "{sw}x{sh}: no black margin left of the picture");
            }
            if y >= 2 {
                assert!(at(x + w / 2, y - 2) < 30, "{sw}x{sh}: no black margin above the picture");
            }
        }
    }
}
