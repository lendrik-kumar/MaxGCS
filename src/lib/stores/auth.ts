// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Session-only unlock state for the Settings password gate. Deliberately NOT persisted (unlike
// settings.ts/layout.ts) — every app restart re-locks, since the whole point is to gate access
// when a device/session changes hands. The password hash itself lives in settings.ts.

import { writable } from 'svelte/store';

export interface AuthState {
  unlocked: boolean;
}

function createAuth() {
  const { subscribe, set } = writable<AuthState>({ unlocked: false });

  return {
    subscribe,
    unlock() { set({ unlocked: true }); },
    lock() { set({ unlocked: false }); },
  };
}

export const auth = createAuth();
