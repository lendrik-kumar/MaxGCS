// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Fixed 1920×1080 video canvas — the geometry. Every picture is shown at the centre of a black
// 1920×1080 canvas at its own size: never stretched, cropped or upscaled; only a picture larger than
// the canvas is scaled down (aspect preserved) until it fits. The backend does this to the frames it
// encodes (live MJPEG + recordings, `src-tauri/src/video/canvas.rs` — keep `canvasPlacement` in step
// with its `placement`); this module does the same placement for the surfaces whose picture is decoded
// by the WebView itself (camera / WebRTC), where there is no encode step to pad.
//
// Pure (no imports) so it can be tested on its own — see `videoCanvas.test.mjs`.

export const CANVAS_W = 1920;
export const CANVAS_H = 1080;
export const CANVAS_ASPECT = CANVAS_W / CANVAS_H;

export interface CanvasRect {
  w: number;
  h: number;
  x: number;
  y: number;
}

/** Where a `srcW`×`srcH` picture lands on the canvas, in canvas pixels. Mirrors the backend. */
export function canvasPlacement(srcW: number, srcH: number): CanvasRect {
  if (!(srcW > 0) || !(srcH > 0)) return { w: 0, h: 0, x: 0, y: 0 };
  let w = srcW;
  let h = srcH;
  if (srcW > CANVAS_W || srcH > CANVAS_H) {
    // Scale down by the one factor that makes it fit (the axis that overshoots most binds).
    if (srcW * CANVAS_H >= srcH * CANVAS_W) {
      w = CANVAS_W;
      h = Math.max(1, Math.floor((CANVAS_W * srcH) / srcW));
    } else {
      h = CANVAS_H;
      w = Math.max(1, Math.floor((CANVAS_H * srcW) / srcH));
    }
  }
  // Centred; offsets rounded down to even pixels like ffmpeg's `pad` (chroma alignment).
  return { w, h, x: Math.floor((CANVAS_W - w) / 2) & ~1, y: Math.floor((CANVAS_H - h) / 2) & ~1 };
}

/** The largest 16:9 canvas box that fits inside a `boxW`×`boxH` surface (CSS `contain`). */
export function canvasBoxFit(boxW: number, boxH: number): { w: number; h: number } {
  const w = Math.min(boxW, boxH * CANVAS_ASPECT);
  return { w, h: w / CANVAS_ASPECT };
}

/** Size and position of the picture inside a `boxW`×`boxH` surface that shows the whole canvas, in
 *  surface pixels. The canvas is fitted into the surface; the picture keeps its place on the canvas. */
export function pictureInSurface(
  srcW: number,
  srcH: number,
  boxW: number,
  boxH: number,
): { w: number; h: number; left: number; top: number } | null {
  const p = canvasPlacement(srcW, srcH);
  if (p.w === 0 || !(boxW > 0) || !(boxH > 0)) return null;
  const c = canvasBoxFit(boxW, boxH);
  const k = c.w / CANVAS_W;
  return {
    w: p.w * k,
    h: p.h * k,
    left: (boxW - c.w) / 2 + p.x * k,
    top: (boxH - c.h) / 2 + p.y * k,
  };
}
