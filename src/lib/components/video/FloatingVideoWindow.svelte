<!--
  SPDX-License-Identifier: GPL-3.0-or-later
  Copyright (C) 2026 Marc Hoffmann (b14ckyy)
-->

<script lang="ts">
  // Floating video window — a chromeless in-app overlay sink for the video router.
  //  • snaps to the bottom-left corner (above the status bar; the bottom widget dock reflows out of
  //    the way — handled in +page.svelte) or floats freely
  //  • drag the video body to move (away from the corner un-snaps; dropping near the corner re-snaps)
  //  • TOP-RIGHT corner grip resizes (aspect-locked, 10–80 % of vh, touch-friendly)
  //  • TOP-LEFT ✕ closes (swaps back first if it was primary); next to it, ⤢ toggles between the
  //    current size and the largest that fits (for a TV / big display), and ⇄ swaps with the map
  //  • double-click the video to swap it with the map (→ videoPrimary) — same as the ⇄ button
  //
  // No title bar (space is precious on a flight display). Layering: separate absolutely-positioned
  // layers share the page stacking context (the .float-win wrapper has no z-index). The map (rendered
  // top-level in +page when swapped) composes between the frosted background (z 120) and the corner
  // controls (z 122), so the mini-map stays interactive while close/resize stay usable. Pinned above
  // the telemetry dock (.zone-bottom-dock, z 100) so a floating window dragged over the dock still
  // shows on top of it, but below NavRail panels (PanelShell, z 150/160) and the toolbar/status-bar
  // chrome (z 200).
  import { t } from 'svelte-i18n';
  import {
    videoStream,
    videoState,
    videoDisplayAspect,
    bindVideoEl,
    setFloatPos,
    setFloatSnapped,
    setFloatHeightFrac,
    setMapLocation,
    toggleFloating,
    toggleFloatExpanded,
    isFloatExpanded,
    clampFloatFrac,
    floatWindowSize,
    FLOAT_SNAP_BOTTOM,
    FLOAT_TOP_SAFE,
    reportMjpegError,
  } from '$lib/stores/video';
  import { canvasSink, mjpegSink } from '$lib/controllers/mjpegSink';
  import VideoReconnectOverlay from '$lib/components/video/VideoReconnectOverlay.svelte';

  // The page's UI scale (1 = 100 %). This window sits inside the `.ui-scale` layer, which is scaled up
  // with a CSS transform, so its coordinates are logical px: the logical viewport is the real one
  // divided by the scale. Without this a large window would overflow a scaled-up (TV) display.
  let { uiScale = 1 }: { uiScale?: number } = $props();

  // True while the map occupies this floating frame (so this window shows the map, not video).
  const mapHere = $derived($videoState.mapLocation === 'floating');

  let vw = $state(typeof window !== 'undefined' ? window.innerWidth : 1280);
  let vh = $state(typeof window !== 'undefined' ? window.innerHeight : 720);
  const lvw = $derived(vw / uiScale);
  const lvh = $derived(vh / uiScale);

  let videoEl = $state<HTMLVideoElement | null>(null);
  $effect(() => {
    bindVideoEl(videoEl, $videoStream);
  });

  const MARGIN = 8;
  const SNAP_THRESHOLD = 56;

  let floatWinEl = $state<HTMLDivElement | null>(null);

  // The resolved display shape (the Display ratio setting + analog-SD correction), not the raw pixel
  // ratio — the picture below is stretched to exactly this shape.
  const aspect = $derived($videoDisplayAspect);
  // Size/limits live in the video store (floatWindowSize) so +page's map-in-frame overlay, which has to
  // line up with this window exactly, uses the very same numbers.
  const size = $derived(floatWindowSize($videoState.floatHeightFrac, aspect, lvw, lvh));
  const height = $derived(size.h);
  const width = $derived(size.w);
  const left = $derived($videoState.floatSnapped ? MARGIN : $videoState.floatX);
  const top = $derived($videoState.floatSnapped ? lvh - height - FLOAT_SNAP_BOTTOM : $videoState.floatY);
  const expanded = $derived(isFloatExpanded($videoState.floatHeightFrac, lvh));
  // The extra buttons sit beside the ✕; skip them on a window too narrow to fit them clear of the grip.
  const showExtraButtons = $derived(width >= 130);

  // This ✕ only shows while the window holds video → it closes the floating window. (When the map is
  // in the frame, +page renders its own ✕ on top that sends the map back to the main view instead.)
  function closeWindow() {
    toggleFloating();
  }

  // ── Drag (from the video body) ─────────────────────────────────────
  let pendingDrag = false;
  let moved = false;
  let startX = 0;
  let startY = 0;
  let baseLeft = 0;
  let baseTop = 0;

  function onBodyPointerDown(e: PointerEvent) {
    if ((e.target as HTMLElement).closest('.fw-corner')) return; // let the corner controls handle it
    pendingDrag = true;
    moved = false;
    startX = e.clientX;
    startY = e.clientY;
    baseLeft = left;
    baseTop = top;
    window.addEventListener('pointermove', onDragMove);
    window.addEventListener('pointerup', onDragUp);
  }
  function onDragMove(e: PointerEvent) {
    if (!pendingDrag) return;
    // Pointer deltas are real px; the window's coordinates are logical (÷ uiScale).
    const dx = (e.clientX - startX) / uiScale;
    const dy = (e.clientY - startY) / uiScale;
    if (!moved && Math.hypot(dx, dy) < 4) return;
    if (!moved) {
      moved = true;
      setFloatSnapped(false); // first real movement detaches from the corner
    }
    const nx = Math.max(0, Math.min(baseLeft + dx, lvw - width));
    const ny = Math.max(0, Math.min(baseTop + dy, lvh - height));
    setFloatPos(nx, ny);
  }
  function onDragUp() {
    window.removeEventListener('pointermove', onDragMove);
    window.removeEventListener('pointerup', onDragUp);
    if (!pendingDrag) return;
    pendingDrag = false;
    if (!moved) return;
    // Re-snap if dropped near the bottom-left corner.
    const nearLeft = $videoState.floatX <= MARGIN + SNAP_THRESHOLD;
    const nearBottom = $videoState.floatY + height >= lvh - FLOAT_SNAP_BOTTOM - SNAP_THRESHOLD;
    if (nearLeft && nearBottom) setFloatSnapped(true);
  }

  // ── Resize (top-right handle) ──────────────────────────────────────
  // Dragging the top-right corner grows the window up + right with the bottom-left anchored:
  // up = bigger, and for a free (un-snapped) window we move the top so the bottom edge stays put.
  let resizing = false;
  let resizeStartY = 0;
  let startFrac = 0;
  let startBottom = 0;
  let startSnapped = false;
  function onResizePointerDown(e: PointerEvent) {
    e.stopPropagation();
    resizing = true;
    resizeStartY = e.clientY;
    startFrac = $videoState.floatHeightFrac;
    startBottom = top + height;
    startSnapped = $videoState.floatSnapped;
    window.addEventListener('pointermove', onResizeMove);
    window.addEventListener('pointerup', onResizeUp);
  }
  function onResizeMove(e: PointerEvent) {
    if (!resizing) return;
    const delta = (resizeStartY - e.clientY) / vh; // drag up → larger (a fraction of the screen height)
    // Clamped to the px floor (4 mini-map buttons) and to what fits below the toolbar.
    const newFrac = clampFloatFrac(startFrac + delta, lvh);
    setFloatHeightFrac(newFrac);
    if (!startSnapped) {
      // Keep the bottom edge fixed (top-right grip): top = bottom − newHeight, but never under the
      // toolbar, and pull the window back in if growing pushed its right edge off-screen.
      const next = floatWindowSize(newFrac, aspect, lvw, lvh);
      setFloatPos(
        Math.max(0, Math.min($videoState.floatX, lvw - next.w)),
        Math.max(FLOAT_TOP_SAFE, startBottom - next.h),
      );
    }
  }
  function onResizeUp() {
    resizing = false;
    window.removeEventListener('pointermove', onResizeMove);
    window.removeEventListener('pointerup', onResizeUp);
  }

  // ── Mini-map move (videoPrimary) — right mouse / two-finger ─────────
  // In primary mode the frame holds the interactive map, so left-drag/single-touch pan the map. To
  // MOVE the frame we grab with the right mouse button (desktop) or two fingers (touch); pinch-zoom
  // is sacrificed there (the zoom buttons + follow mode cover it). Window-level capture so we run
  // before Leaflet and can stop it seeing the gesture.
  let fmActive = false;
  let fmStartX = 0;
  let fmStartY = 0;
  let fmBaseLeft = 0;
  let fmBaseTop = 0;
  let fmMoved = false;

  function pointInFrame(cx: number, cy: number): boolean {
    const r = floatWinEl?.getBoundingClientRect();
    return !!r && cx >= r.left && cx <= r.right && cy >= r.top && cy <= r.bottom;
  }
  function frameMoveStart(cx: number, cy: number) {
    fmActive = true;
    fmMoved = false;
    fmStartX = cx;
    fmStartY = cy;
    fmBaseLeft = left;
    fmBaseTop = top;
  }
  function frameMoveTo(cx: number, cy: number) {
    if (!fmActive) return;
    const dx = (cx - fmStartX) / uiScale;
    const dy = (cy - fmStartY) / uiScale;
    if (!fmMoved && Math.hypot(dx, dy) < 4) return;
    if (!fmMoved) {
      fmMoved = true;
      setFloatSnapped(false);
    }
    setFloatPos(
      Math.max(0, Math.min(fmBaseLeft + dx, lvw - width)),
      Math.max(0, Math.min(fmBaseTop + dy, lvh - height)),
    );
  }
  function frameMoveEnd() {
    if (!fmActive) return;
    fmActive = false;
    if (!fmMoved) return;
    const nearLeft = $videoState.floatX <= MARGIN + SNAP_THRESHOLD;
    const nearBottom = $videoState.floatY + height >= lvh - FLOAT_SNAP_BOTTOM - SNAP_THRESHOLD;
    if (nearLeft && nearBottom) setFloatSnapped(true);
  }

  // Narrow $derived, deliberately not a raw `$videoState.floating` read inside the effect: that would
  // tie the effect to the WHOLE video store, and the handlers below write to it (setFloatPos /
  // setFloatSnapped) — so all seven window listeners were torn down and re-registered on every single
  // pointermove of a drag (and on every unrelated store patch, e.g. a reconnect-attempt tick).
  const gestureCapture = $derived($videoState.floating && mapHere);
  $effect(() => {
    if (!gestureCapture) return;
    const mid = (t: TouchList) => ({
      x: (t[0].clientX + t[1].clientX) / 2,
      y: (t[0].clientY + t[1].clientY) / 2,
    });
    const onCtx = (e: MouseEvent) => {
      if (pointInFrame(e.clientX, e.clientY)) e.preventDefault(); // no context menu over the frame
    };
    const onPD = (e: PointerEvent) => {
      if (e.button === 2 && pointInFrame(e.clientX, e.clientY)) {
        e.preventDefault();
        e.stopPropagation();
        frameMoveStart(e.clientX, e.clientY);
      }
    };
    const onPM = (e: PointerEvent) => {
      if (fmActive) frameMoveTo(e.clientX, e.clientY);
    };
    const onPU = () => frameMoveEnd();
    const onTS = (e: TouchEvent) => {
      if (e.touches.length === 2) {
        const m = mid(e.touches);
        if (pointInFrame(m.x, m.y)) {
          e.preventDefault();
          e.stopPropagation();
          frameMoveStart(m.x, m.y);
        }
      }
    };
    const onTM = (e: TouchEvent) => {
      if (fmActive && e.touches.length >= 2) {
        e.preventDefault();
        e.stopPropagation();
        const m = mid(e.touches);
        frameMoveTo(m.x, m.y);
      }
    };
    const onTE = (e: TouchEvent) => {
      if (fmActive && e.touches.length < 2) frameMoveEnd();
    };
    window.addEventListener('contextmenu', onCtx, true);
    window.addEventListener('pointerdown', onPD, true);
    window.addEventListener('pointermove', onPM, true);
    window.addEventListener('pointerup', onPU, true);
    window.addEventListener('touchstart', onTS, { capture: true, passive: false });
    window.addEventListener('touchmove', onTM, { capture: true, passive: false });
    window.addEventListener('touchend', onTE, true);
    return () => {
      window.removeEventListener('contextmenu', onCtx, true);
      window.removeEventListener('pointerdown', onPD, true);
      window.removeEventListener('pointermove', onPM, true);
      window.removeEventListener('pointerup', onPU, true);
      window.removeEventListener('touchstart', onTS, true);
      window.removeEventListener('touchmove', onTM, true);
      window.removeEventListener('touchend', onTE, true);
    };
  });
</script>

<svelte:window bind:innerWidth={vw} bind:innerHeight={vh} />

{#if $videoState.floating}
  <!-- No z-index on the wrapper → no stacking context; layers compose with the top-level map. -->
  <div bind:this={floatWinEl} class="float-win" style="left:{left}px; top:{top}px; width:{width}px; height:{height}px;">
    <!-- frame background (behind) — border + shadow only; the video/map covers it (object-fit: cover) -->
    <div class="fw-bg"></div>

    <!-- content: the video. When the map is in this frame, it's rendered (top-level) by +page here
         instead, and the body is omitted. Double-click the video → the map jumps into this frame. -->
    {#if !mapHere}
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="fw-body" onpointerdown={onBodyPointerDown} ondblclick={() => setMapLocation('floating')}>
        {#if $videoState.status === 'live' && $videoState.mjpegUrl}
          <!-- Native / MJPEG feed (no MediaStream): drawn by the off-thread reader where the WebView
               allows it, otherwise the plain <img> multipart stream. -->
          {#if $canvasSink}
            <canvas use:mjpegSink class:mirror={$videoState.mirror}></canvas>
          {:else}
            <!-- svelte-ignore a11y_missing_attribute -->
            <img src={$videoState.mjpegUrl} class:mirror={$videoState.mirror} onerror={reportMjpegError} />
          {/if}
        {:else if $videoState.status === 'live'}
          <!-- svelte-ignore a11y_media_has_caption -->
          <video bind:this={videoEl} autoplay muted playsinline class:mirror={$videoState.mirror}></video>
        {:else}
          <div class="fw-ph">{$videoState.status === 'starting' ? $t('video.starting') : $t('video.off')}</div>
        {/if}
        <VideoReconnectOverlay />
      </div>
    {/if}

    <!-- Corner controls (video mode only). When the map fills this frame, the map (a separate unzoomed
         top-level layer) covers these, so +page renders the equivalents above it. -->
    {#if !mapHere}
      <!-- close (top-left) — overlay, touch-sized -->
      <button class="fw-corner fw-close" onclick={closeWindow} title={$t('video.close')}>✕</button>

      {#if showExtraButtons}
        <!-- expand / restore — toggles between the current size and the largest that fits -->
        <button
          class="fw-corner fw-btn fw-expand"
          onclick={() => toggleFloatExpanded(lvw, lvh)}
          title={expanded ? $t('video.restoreWindow') : $t('video.expandWindow')}
          aria-label={expanded ? $t('video.restoreWindow') : $t('video.expandWindow')}
        >
          <svg viewBox="0 0 24 24" width="14" height="14" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
            {#if expanded}
              <path d="M20 10h-6V4M4 14h6v6M14 10l7-7M10 14l-7 7" />
            {:else}
              <path d="M14 4h6v6M10 20H4v-6M20 4l-7 7M4 20l7-7" />
            {/if}
          </svg>
        </button>
        <!-- swap with the map (same as double-clicking the video) -->
        <button
          class="fw-corner fw-btn fw-swap"
          onclick={() => setMapLocation('floating')}
          title={$t('video.swapWithMap')}
          aria-label={$t('video.swapWithMap')}
        >
          <svg viewBox="0 0 24 24" width="14" height="14" fill="none" stroke="currentColor" stroke-width="2.2" stroke-linecap="round" stroke-linejoin="round" aria-hidden="true">
            <path d="M7 7h13M16 3l4 4-4 4M17 17H4M8 13l-4 4 4 4" />
          </svg>
        </button>
      {/if}

      <!-- resize grip (top-right) — visible, touch-sized -->
      <!-- svelte-ignore a11y_no_static_element_interactions -->
      <div class="fw-corner fw-resize" onpointerdown={onResizePointerDown} title="Resize"></div>
    {/if}
  </div>
{/if}

<style>
  .float-win {
    position: absolute;
    /* No z-index on purpose (see script header). */
    pointer-events: none; /* layers opt back in individually */
  }
  /* Frame background — border + shadow only. No backdrop-filter: the video/map always covers this
     layer (object-fit: cover), so the blur was never visible, and on WebKitGTK it triggered a
     compositing artifact (a flickering blurry mini-copy) over the <img> MJPEG-fallback feed. */
  .fw-bg {
    position: absolute;
    inset: 0;
    z-index: 120;
    pointer-events: none;
    background: rgba(46, 46, 46, 0.92);
    border: 1px solid var(--mx-red-dim, rgba(224, 48, 44, 0.35));
    border-radius: 8px;
    box-shadow: 0 4px 20px rgba(0, 0, 0, 0.4);
  }
  .fw-body {
    position: absolute;
    inset: 0;
    z-index: 121;
    pointer-events: auto;
    background: #000;
    overflow: hidden;
    border-radius: 8px;
    cursor: grab;
    touch-action: none; /* let pointer-drag move the window instead of scrolling/panning */
  }
  .fw-body:active {
    cursor: grabbing;
  }
  .fw-body video,
  .fw-body img,
  .fw-body canvas {
    width: 100%;
    height: 100%;
    /* The window is sized to the display shape (see floatWindowSize), so `fill` shows the whole
       picture at that shape — unlike `cover`, which cropped it and could not correct non-square
       pixels (analog SD). */
    object-fit: fill;
    display: block;
    /* Own compositing layer — see VideoWidget: keeps the 60 fps MJPEG <img> from dirtying shared
       layer tiles every frame on WebKitGTK. */
    will-change: transform;
  }
  .fw-body video.mirror,
  .fw-body img.mirror,
  .fw-body canvas.mirror {
    transform: scaleX(-1);
  }
  .fw-ph {
    position: absolute;
    inset: 0;
    display: flex;
    align-items: center;
    justify-content: center;
    color: #888;
    font-size: 12px;
  }

  /* Corner controls — overlay the content corners (no title bar); touch-sized. */
  .fw-corner {
    position: absolute;
    top: 0;
    width: 26px;
    height: 26px;
    z-index: 122;
    pointer-events: auto;
    box-sizing: border-box;
    touch-action: none;
  }
  .fw-close {
    left: 0;
    display: flex;
    align-items: center;
    justify-content: center;
    font-size: 13px;
    line-height: 1;
    color: #e0e0e0;
    background: rgba(0, 0, 0, 0.45);
    border: none;
    border-radius: 8px 0 8px 0;
    cursor: pointer;
  }
  .fw-close:hover {
    background: rgba(212, 0, 0, 0.7);
    color: #fff;
  }
  /* Expand/restore + swap buttons — a strip continuing right of the ✕. */
  .fw-btn {
    display: flex;
    align-items: center;
    justify-content: center;
    padding: 0;
    color: #e0e0e0;
    background: rgba(0, 0, 0, 0.45);
    border: none;
    border-left: 1px solid rgba(255, 255, 255, 0.12);
    border-radius: 0;
    cursor: pointer;
  }
  .fw-btn:hover {
    background: rgba(224, 48, 44, 0.7);
    color: #fff;
  }
  .fw-expand {
    left: 26px;
  }
  .fw-swap {
    left: 52px;
    border-radius: 0 0 8px 0;
  }
  .fw-resize {
    right: 0;
    cursor: nesw-resize;
    border-radius: 0 8px 0 8px;
    /* visible grab affordance in the top-right corner */
    background: linear-gradient(225deg, rgba(224, 48, 44, 0.85) 42%, transparent 42%);
  }
  .fw-resize:hover {
    background: linear-gradient(225deg, rgba(224, 48, 44, 1) 50%, transparent 50%);
  }
</style>
