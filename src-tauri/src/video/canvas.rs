// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

//! Fixed 1920×1080 output. Every frame that leaves the backend video pipeline — the live MJPEG feed
//! and the recorded leg — is **scaled to fit a 1920×1080 canvas and centred on it**:
//!
//! * the picture keeps its aspect ratio — no stretching and no cropping;
//! * it is scaled **up or down** by the one factor that makes it fill the canvas on its limiting axis,
//!   so a 720×480 camera becomes 1620×1080 and a 1280×720 one becomes the full 1920×1080 (this is the
//!   "enhanced to 1080p" output the GCS requirement asks for — placing a small picture at its own size
//!   on a big black canvas just shrinks it on screen);
//! * whatever the scaled picture does not cover is black padding (bars on the sides for 4:3 / 3:2
//!   sources, none for 16:9).
//!
//! The work is one ffmpeg filter chain (`scale` + `pad` + `setsar`), so it adds no process, no pipe and
//! no per-frame copy through this program. `placement` mirrors the chain's arithmetic and is what the
//! tests (and any caller that needs the rectangle) use.

/// Canvas width in pixels.
pub const CANVAS_W: u32 = 1920;
/// Canvas height in pixels.
pub const CANVAS_H: u32 = 1080;

/// Where a `src_w`×`src_h` picture lands on the canvas: `(width, height, x, y)` of the scaled picture.
///
/// Mirrors the filter chain below: the picture is scaled (up or down) to the largest size that fits
/// the canvas with its aspect ratio kept — the binding axis takes the full canvas length, the other is
/// derived and rounded to the nearest pixel, as ffmpeg's `scale` does — then centred, with the offset
/// rounded down to an even pixel (chroma-subsampling alignment, which is what `pad` does).
pub fn placement(src_w: u32, src_h: u32) -> (u32, u32, u32, u32) {
    if src_w == 0 || src_h == 0 {
        return (0, 0, 0, 0);
    }
    let (sw, sh) = (src_w as u64, src_h as u64);
    let (cw, ch) = (CANVAS_W as u64, CANVAS_H as u64);
    let round_div = |n: u64, d: u64| ((n + d / 2) / d) as u32;
    // The axis with the smaller `canvas / source` ratio is the one that binds.
    let (w, h) = if cw * sh <= ch * sw {
        (CANVAS_W, round_div(cw * sh, sw).clamp(1, CANVAS_H))
    } else {
        (round_div(ch * sw, sh).clamp(1, CANVAS_W), CANVAS_H)
    };
    let x = ((CANVAS_W - w) / 2) & !1;
    let y = ((CANVAS_H - h) / 2) & !1;
    (w, h, x, y)
}

/// The ffmpeg filter chain that scales the input to fit the canvas and centres it on black, as a single
/// comma-joined chain (usable after `-vf`, or as the body of a `-filter_complex` label).
///
/// * `scale` fits the picture inside 1920×1080 (`force_original_aspect_ratio=decrease` keeps the
///   aspect ratio and picks the largest size that fits, enlarging a small picture as well as shrinking
///   a big one) — bicubic, ffmpeg's default and cheap enough for a live 1080p feed;
/// * `pad` centres it on the canvas with black;
/// * `setsar=1` states that the result has square pixels, so a player does not stretch the picture a
///   second time (analog SD sources carry a non-square sample aspect ratio).
pub fn filter() -> String {
    format!(
        "scale=w={w}:h={h}:force_original_aspect_ratio=decrease,\
         pad={w}:{h}:(ow-iw)/2:(oh-ih)/2:color=black,setsar=1",
        w = CANVAS_W,
        h = CANVAS_H,
    )
}

/// `filter()` plus, when `fps` is `Some(n)`, a trailing `fps=n` that makes the output a constant `n`
/// frames per second — a slower source has frames repeated (it can't invent detail between them), a
/// faster one has frames dropped. Last in the chain on purpose: the repeats are then cheap references to
/// the already scaled frame instead of being scaled again, and every leg (live MJPEG + recording) keeps
/// the same cadence.
pub fn filter_fps(fps: Option<u32>) -> String {
    match fps {
        Some(n) if n > 0 => format!("{},fps={n}", filter()),
        _ => filter(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fps_is_appended_last_and_only_when_asked() {
        assert_eq!(filter_fps(None), filter());
        assert_eq!(filter_fps(Some(0)), filter());
        let f = filter_fps(Some(60));
        assert!(f.starts_with(&filter()) && f.ends_with(",fps=60"), "{f}");
    }

    #[test]
    fn small_pictures_are_enlarged_to_fill_the_canvas_on_their_limiting_axis() {
        // Analog SD (3:2 pixels) and 4:3: height binds → pillarboxed at the full 1080 lines.
        assert_eq!(placement(720, 480), (1620, 1080, 150, 0));
        assert_eq!(placement(640, 480), (1440, 1080, 240, 0));
        // 16:9 sources of any size fill the whole canvas.
        assert_eq!(placement(1280, 720), (1920, 1080, 0, 0));
        assert_eq!(placement(640, 360), (1920, 1080, 0, 0));
        assert_eq!(placement(1920, 1080), (1920, 1080, 0, 0));
    }

    #[test]
    fn wide_and_tall_pictures_bar_only_the_free_axis() {
        // 2.4:1 → width binds, letterboxed; 9:16 portrait → height binds, pillarboxed.
        assert_eq!(placement(1920, 800), (1920, 800, 0, 140));
        assert_eq!(placement(1080, 1920), (608, 1080, 656, 0));
    }

    #[test]
    fn oversized_pictures_shrink_to_fit() {
        assert_eq!(placement(3840, 2160), (1920, 1080, 0, 0));
        let (w, h, x, y) = placement(4096, 2160); // 17:9 → width binds
        assert_eq!((w, x), (1920, 0));
        assert_eq!(h, 1013); // 1012.5 rounds to the nearest pixel, as ffmpeg's scale does
        assert_eq!(y, 32);
    }

    #[test]
    fn placement_always_fills_the_canvas_on_one_axis_and_keeps_the_aspect_ratio() {
        for &(sw, sh) in &[(1, 1), (2, 2), (321, 241), (719, 479), (720, 480), (1919, 1079), (1921, 1081), (7680, 4320), (100, 5000)] {
            let (w, h, x, y) = placement(sw, sh);
            assert!(w >= 1 && h >= 1, "{sw}x{sh} collapsed");
            assert!(x + w <= CANVAS_W && y + h <= CANVAS_H, "{sw}x{sh} overflows the canvas");
            assert!(w == CANVAS_W || h == CANVAS_H, "{sw}x{sh} leaves slack on both axes: {w}x{h}");
            // Aspect ratio kept to within the rounding of one pixel on the derived axis.
            let want = sw as f64 / sh as f64;
            let got = w as f64 / h as f64;
            let tol = (1.0 / h.min(w) as f64) * 2.0 * want.max(1.0);
            assert!((want - got).abs() <= tol.max(0.01), "{sw}x{sh} aspect {want} became {got}");
            // Centred to within the even-pixel alignment plus the odd-size remainder (3 px at most).
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
    fn filter_fits_the_fixed_canvas_and_pads_it() {
        let f = filter();
        assert!(f.starts_with("scale=w=1920:h=1080:force_original_aspect_ratio=decrease,"), "{f}");
        assert!(f.contains("pad=1920:1080:(ow-iw)/2:(oh-ih)/2:color=black"), "{f}");
        assert!(f.ends_with("setsar=1"), "{f}");
        // One chain: no stray whitespace/newlines from the line continuation.
        assert!(!f.contains(char::is_whitespace), "{f}");
    }

    /// Runs the real ffmpeg when one is installed (skipped silently otherwise — CI machines and fresh
    /// checkouts usually have none): renders a white test picture through `filter()` and checks the
    /// raw output is exactly 1920×1080, with the white picture filling `placement`'s rectangle and
    /// black everywhere outside it.
    #[test]
    fn real_ffmpeg_output_is_a_scaled_centred_1080p_canvas() {
        let Some(ff) = crate::video::ffmpeg::find_ffmpeg() else {
            eprintln!("ffmpeg not found — skipping the end-to-end canvas check");
            return;
        };
        for &(sw, sh) in &[(640u32, 360u32), (720, 480), (640, 480), (1280, 720), (2560, 1440), (321, 241)] {
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
            // A few pixels inside the rectangle (placement may differ from ffmpeg's rounding by one
            // pixel on an edge): white. The picture really was enlarged — not left at its own size.
            const M: u32 = 4;
            assert!(at(x + M, y + M) > 200, "{sw}x{sh}: picture missing at its top-left");
            assert!(at(x + w - M - 1, y + h - M - 1) > 200, "{sw}x{sh}: picture missing at its bottom-right");
            assert!(at(x + w / 2, y + h / 2) > 200, "{sw}x{sh}: picture missing at its centre");
            // The canvas corners are black unless the picture reaches them (a full-canvas fill).
            for &(cx, cy) in &[(0, 0), (CANVAS_W - 1, 0), (0, CANVAS_H - 1), (CANVAS_W - 1, CANVAS_H - 1)] {
                if cx + M < x || cx >= x + w + M || cy + M < y || cy >= y + h + M {
                    assert!(at(cx, cy) < 30, "{sw}x{sh}: corner ({cx},{cy}) is not black");
                }
            }
            // And a clear margin outside the picture is black where there is room for one.
            if x >= 2 * M {
                assert!(at(x - M, y + h / 2) < 30, "{sw}x{sh}: no black bar left of the picture");
                assert!(at(x + w + M, y + h / 2) < 30, "{sw}x{sh}: no black bar right of the picture");
            }
            if y >= 2 * M {
                assert!(at(x + w / 2, y - M) < 30, "{sw}x{sh}: no black bar above the picture");
                assert!(at(x + w / 2, y + h + M) < 30, "{sw}x{sh}: no black bar below the picture");
            }
        }
    }
}
