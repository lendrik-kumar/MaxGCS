// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

import { writable } from 'svelte/store';

// ── Layout profiles ────────────────────────────────────────────
// Each profile defines default zone visibility + optional size
// overrides.  Profiles can be switched at runtime (e.g. when
// entering mission-planning mode).

export type LayoutProfile = 'flight' | 'mission' | 'area-planner';

export interface ZoneDock {
  visible: boolean;
  /** CSS length override, e.g. "clamp(200px, 30vh, 400px)".
   *  `null` = use the default from the grid definition. */
  sizeOverride: string | null;
}

export interface LayoutState {
  profile: LayoutProfile;
  /** Bottom dock – height is overridable */
  bottomDock: ZoneDock;
  /** Side dock – width is overridable */
  sideDock: ZoneDock;
}

// ── Profile presets ────────────────────────────────────────────

const profilePresets: Record<LayoutProfile, Omit<LayoutState, 'profile'>> = {
  flight: {
    bottomDock: { visible: true, sizeOverride: null },
    sideDock:   { visible: true, sizeOverride: null },
  },
  mission: {
    bottomDock: { visible: true, sizeOverride: 'clamp(200px, 30vh, 400px)' },
    sideDock:   { visible: false, sizeOverride: null },
  },
  'area-planner': {
    bottomDock: { visible: true, sizeOverride: 'clamp(200px, 25vh, 350px)' },
    sideDock:   { visible: false, sizeOverride: null },
  },
};

// ── Default grid sizes (used when no override is set) ──────────

export const GRID_DEFAULTS = {
  navRailWidth:     '62px',
  toolbarHeight:    '53px',
  statusBarHeight:  '24px',
  bottomDockHeight: 'clamp(184px, 20vh, 300px)',
  sideDockWidth:    'clamp(150px, 15vw, 250px)',
} as const;

// ── Persistence ────────────────────────────────────────────────
// Only the user's dragged size overrides survive a restart — profile/visibility
// stay session-only (a profile switch always resets to its preset, by design).

const STORAGE_KEY = 'kite-gc-layout';

interface PersistedLayout {
  bottomSizeOverride: string | null;
  sideSizeOverride: string | null;
}

function loadPersisted(): PersistedLayout {
  try {
    const raw = localStorage.getItem(STORAGE_KEY);
    if (raw) {
      const parsed = JSON.parse(raw) as Partial<PersistedLayout>;
      return {
        bottomSizeOverride: parsed.bottomSizeOverride ?? null,
        sideSizeOverride: parsed.sideSizeOverride ?? null,
      };
    }
  } catch {
    // Ignore parse errors, use defaults
  }
  return { bottomSizeOverride: null, sideSizeOverride: null };
}

function savePersisted(state: LayoutState) {
  const toSave: PersistedLayout = {
    bottomSizeOverride: state.bottomDock.sizeOverride,
    sideSizeOverride: state.sideDock.sizeOverride,
  };
  localStorage.setItem(STORAGE_KEY, JSON.stringify(toSave));
}

// ── Store ──────────────────────────────────────────────────────

function createLayout() {
  const persisted = loadPersisted();
  const store = writable<LayoutState>({
    profile: 'flight',
    ...profilePresets.flight,
    bottomDock: { ...profilePresets.flight.bottomDock, sizeOverride: persisted.bottomSizeOverride },
    sideDock: { ...profilePresets.flight.sideDock, sizeOverride: persisted.sideSizeOverride },
  });

  return {
    subscribe: store.subscribe,

    /** Switch to a named profile (resets overrides to preset). */
    setProfile(profile: LayoutProfile) {
      store.set({ profile, ...profilePresets[profile] });
    },

    /** Toggle bottom dock visibility. */
    setBottomDockVisible(visible: boolean) {
      store.update(s => ({ ...s, bottomDock: { ...s.bottomDock, visible } }));
    },

    /** Toggle side dock visibility. */
    setSideDockVisible(visible: boolean) {
      store.update(s => ({ ...s, sideDock: { ...s.sideDock, visible } }));
    },

    /** Override bottom dock height (CSS length or null for default). Persisted across restarts. */
    setBottomDockHeight(height: string | null) {
      store.update(s => {
        const next = { ...s, bottomDock: { ...s.bottomDock, sizeOverride: height } };
        savePersisted(next);
        return next;
      });
    },

    /** Override side dock width (CSS length or null for default). Persisted across restarts. */
    setSideDockWidth(width: string | null) {
      store.update(s => {
        const next = { ...s, sideDock: { ...s.sideDock, sizeOverride: width } };
        savePersisted(next);
        return next;
      });
    },
  };
}

export const layout = createLayout();
