// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Lightweight host-OS detection for platform-specific UI (e.g. macOS traffic-light window controls on
// the LEFT vs the Windows/Linux control cluster on the RIGHT). We read the WebView user-agent rather
// than pulling in @tauri-apps/plugin-os: the string is stable per platform (WKWebView reports
// "Macintosh", WebView2 "Windows", WebKitGTK "Linux") and this only drives cosmetic layout, so a sync
// value with no extra dependency / async round-trip is preferable.

const ua = typeof navigator !== 'undefined' ? navigator.userAgent : '';

/** True when running inside the macOS WebView (WKWebView) — used to mirror native window-control placement. */
export const isMacOS = /Macintosh|Mac OS X/i.test(ua);

/** True on the Windows WebView (WebView2) — drives the native-capture backend (DirectShow). */
export const isWindows = /Windows/i.test(ua);

/** True on the Linux WebView (WebKitGTK; excludes Android) — drives the native-capture backend (V4L2). */
export const isLinux = /Linux/i.test(ua) && !/Android/i.test(ua);

/** True on WebKitGTK specifically — the Linux WebView, as opposed to macOS's WKWebView (a different
 *  WebKit port on Core Animation) or Chromium-based WebView2.
 *
 *  Drives the hard-blink indicator mode. On WebKitGTK a looping CSS animation makes the compositor
 *  rebuild the entire window every frame: the cost is per frame produced, not per pixel changed, so a
 *  single 6-pixel dot measured ~46 % of a core. It is unaffected by the element's size, by
 *  `will-change`, by `steps()` (which quantises the value, not the frame production), and even by
 *  whether the element is on screen at all — a marker panned to the other side of the world costs
 *  exactly the same. Only producing fewer frames helps: 5 Hz → 23 %, 2 Hz → 14 %, 1 Hz → 6 %.
 *
 *  Neither WebView2 nor macOS shows this, so both keep the smooth animations. */
export const isWebKitGtk = isLinux;

/** True on any WebKit-based WebView — WebKitGTK on Linux and WKWebView on macOS, which are different
 *  ports of the same WebCore and so share its resource loader.
 *
 *  Drives the `?raw=1` request of the off-thread MJPEG reader. WebKit handles
 *  `multipart/x-mixed-replace` inside that loader and never exposes it to `fetch`: measured on
 *  WebKitGTK 2.52.5 from a `tauri://localhost` page, the response headers arrive and the first
 *  `reader.read()` fails with `Load failed` at zero bytes — main thread and worker alike, which is
 *  what silently pushed Linux back onto the `<img>` sink. The identical bytes under another content
 *  type stream perfectly. WebView2 reads multipart directly and is deliberately left untouched. */
export const isWebKit = isLinux || isMacOS;
