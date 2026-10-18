// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Svelte action for a `<video>` the WebView decodes itself (camera / WebRTC): places the picture at the
// centre of the fixed 1920×1080 canvas instead of stretching it over its surface. The surface shows the
// whole 16:9 canvas; the picture keeps its own pixel size, scaled only by the canvas's own fit into the
// surface — never stretched, cropped or upscaled beyond that.
//
// It only sets inline size/position on the element and removes them again when the canvas is off or the
// size is unknown, so the surface's own CSS (`width/height: 100%`) takes back over. Costs nothing per
// frame: it reacts to size changes (store + ResizeObserver), not to video frames.
//
// The backend-fed surfaces (native / RTSP MJPEG) need none of this — their frames arrive already padded
// to 1920×1080 (`src-tauri/src/video/canvas.rs`).

import { videoCanvasSource } from '$lib/stores/video';
import { pictureInSurface } from '$lib/helpers/videoCanvas';

const PROPS = ['position', 'inset', 'margin', 'width', 'height', 'left', 'top', 'object-fit'] as const;

export function canvasFrame(node: HTMLElement) {
  const parent = node.parentElement;
  let src: { w: number; h: number } | null = null;
  let boxW = parent?.clientWidth ?? 0;
  let boxH = parent?.clientHeight ?? 0;

  function clear() {
    for (const p of PROPS) node.style.removeProperty(p);
  }

  function apply() {
    const r = src ? pictureInSurface(src.w, src.h, boxW, boxH) : null;
    if (!r) {
      clear();
      return;
    }
    const s = node.style;
    s.position = 'absolute';
    s.inset = 'auto';
    s.margin = '0';
    s.left = `${r.left}px`;
    s.top = `${r.top}px`;
    s.width = `${r.w}px`;
    s.height = `${r.h}px`;
    // The element is exactly the picture's shape now; `fill` just avoids any sub-pixel letterboxing.
    s.objectFit = 'fill';
  }

  const unsub = videoCanvasSource.subscribe((v) => {
    src = v;
    apply();
  });

  let ro: ResizeObserver | null = null;
  if (parent && typeof ResizeObserver !== 'undefined') {
    ro = new ResizeObserver(() => {
      const w = parent.clientWidth;
      const h = parent.clientHeight;
      if (w === boxW && h === boxH) return;
      boxW = w;
      boxH = h;
      apply();
    });
    ro.observe(parent);
  }

  return {
    destroy() {
      unsub();
      ro?.disconnect();
      clear();
    },
  };
}
