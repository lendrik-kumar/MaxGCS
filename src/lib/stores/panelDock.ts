// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Viewport-space right edge (px, already accounting for --ui-scale) of the currently open
// left-docked tool panel, or 0 when none is open. Published by PanelShell (ResizeObserver +
// getBoundingClientRect) so frame-pinned overlays — chiefly the STATUSTEXT system-message toasts —
// can centre themselves in the area to the RIGHT of the panel instead of overlapping it (issue #10).

import { writable } from 'svelte/store';

export const panelDockRight = writable(0);
