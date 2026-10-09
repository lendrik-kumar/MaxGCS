// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Svelte action that makes a video surface zoomable: the mouse wheel steps the shared zoom level and,
// where enabled, dragging pans the zoomed picture. It only drives the shared store (`stores/video`);
// the picture itself is scaled by a `.zoom-layer` inside the surface (see `videoZoomTransform`), so
// the same zoom shows on the panel preview, the floating window and the full-screen view at once.

import { get } from 'svelte/store';
import { videoState, stepVideoZoom, panVideoBy } from '$lib/stores/video';

export interface VideoZoomOptions {
  /** Mouse wheel steps the zoom level. Off where the wheel must keep scrolling (the panel preview). */
  wheel?: boolean;
  /** Dragging pans the picture while zoomed in. Off where dragging already means something else (the
   *  floating window moves when dragged). */
  pan?: boolean;
}

export function videoZoom(node: HTMLElement, opts: VideoZoomOptions = {}) {
  const wheel = opts.wheel ?? true;
  const pan = opts.pan ?? false;

  // A touchpad fires a burst of wheel events per gesture and a wheel notch can report several; one
  // preset step per ~150 ms keeps 1× → 2× → 4× controllable instead of jumping straight to the end.
  let lastWheel = 0;
  function onWheel(e: WheelEvent) {
    e.preventDefault();
    e.stopPropagation();
    const now = performance.now();
    if (now - lastWheel < 150) return;
    lastWheel = now;
    stepVideoZoom(e.deltaY < 0 ? 1 : -1);
  }

  let dragging = false;
  let lastX = 0;
  let lastY = 0;
  function onDown(e: PointerEvent) {
    if (e.button !== 0 || get(videoState).zoom <= 1) return;
    if ((e.target as HTMLElement).closest('button, select, input')) return;
    dragging = true;
    lastX = e.clientX;
    lastY = e.clientY;
    // Window-level listeners rather than pointer capture: capture would retarget the click, and the
    // surfaces use double-click on the picture (swap with the map), which must keep working while zoomed.
    window.addEventListener('pointermove', onMove);
    window.addEventListener('pointerup', onUp);
    window.addEventListener('pointercancel', onUp);
  }
  function onMove(e: PointerEvent) {
    if (!dragging) return;
    const z = get(videoState).zoom;
    // Real (viewport) px on both sides, so a scaled-up UI doesn't skew the drag.
    const r = node.getBoundingClientRect();
    const travelX = ((z - 1) * r.width) / 2;
    const travelY = ((z - 1) * r.height) / 2;
    if (travelX > 0 && travelY > 0) panVideoBy((e.clientX - lastX) / travelX, (e.clientY - lastY) / travelY);
    lastX = e.clientX;
    lastY = e.clientY;
  }
  function onUp() {
    dragging = false;
    window.removeEventListener('pointermove', onMove);
    window.removeEventListener('pointerup', onUp);
    window.removeEventListener('pointercancel', onUp);
  }

  if (wheel) node.addEventListener('wheel', onWheel, { passive: false });
  let unsub: (() => void) | undefined;
  if (pan) {
    node.addEventListener('pointerdown', onDown);
    // Grab cursor / no touch-scrolling only while there is something to drag.
    unsub = videoState.subscribe((s) => {
      node.style.cursor = s.zoom > 1 ? 'grab' : '';
      node.style.touchAction = s.zoom > 1 ? 'none' : '';
    });
  }

  return {
    destroy() {
      node.removeEventListener('wheel', onWheel);
      node.removeEventListener('pointerdown', onDown);
      onUp();
      unsub?.();
    },
  };
}
