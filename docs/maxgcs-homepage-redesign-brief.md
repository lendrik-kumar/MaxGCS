# MaxGCS Homepage Redesign — Brief for Claude Code

Repo: `lendrik-kumar/MaxGCS`, branch `mannat`. SvelteKit + Tauri.

This brief already includes reconnaissance done against the real repo, so you
(Claude Code) can skip straight to planning changes instead of re-deriving the
architecture from zero. Verify everything below against the live tree before
touching code — this was current as of the `mannat` branch at the time of
writing, but re-check line numbers and confirm nothing has moved.

## Read first

- `src/routes/+page.svelte` — the entire current homepage. ~3,234 lines. This
  is the orchestrator: imports every panel/widget, holds most of the page
  state, and renders the current map + floating widget overlay + NavRail.
- `docs/user/for-developers/ui-framework.md` — existing docs on the panel/UI
  framework. Read this in full; it likely documents conventions this redesign
  must follow (the code references a `PANEL_FRAMEWORK.md` in comments —
  confirm whether that's this file, a renamed/moved file, or missing, and
  reconcile before relying on it).
- `src/lib/config/widgetRegistry.ts` — canonical list of telemetry widgets,
  their `widgetClass` (`large` / `small` / `wide`), and sizing constants
  (`LARGE_BASE_VMIN`, `SMALL_BASE_VMIN`, `MIN_SCALE`).
- `src/lib/components/WidgetPanel.svelte` (~668 lines) — an existing panel
  strip component with **drag-and-drop widget reordering already built in**.
  Takes `widgetIds`, `orientation` (`horizontal`/`vertical`), `panelId`,
  `onreorder`, `onreceive`. This is very likely the mechanism to reuse for
  arranging HOME/SPD/ALT/GPS/BAT/LINK/MODE/AHI/Compass below the map, rather
  than hand-building a new grid.
- `src/lib/controllers/widgetController.ts` — pure functions operating on
  `PanelConfig` (`reorderPanel`, `receiveWidget`, `toggleWidgetVisibility`,
  `isWidgetActive`, `getWidgetPanel`).
- `src/lib/stores/settings.ts` — `PanelConfig` interface and `defaultPanels`
  (search for `panels:` around line ~428). This is where widget→panel
  assignment is persisted. Understand this before changing panel layout, or
  saved user layouts may break on upgrade.
- `src/lib/components/NavRail.svelte` (~190 lines) — **already exists**, and
  is closer to the target than the original spec assumed. It's currently a
  hamburger-toggle rail: collapsed by default, expands to show tab buttons
  with icon + label on toggle. It is NOT the "large pop-out menu covering the
  screen" the spec describes eliminating — it's a compact rail already. The
  actual gap vs. the spec: buttons are hidden until the hamburger is clicked
  (`{#if open}`), whereas the spec wants them **always visible** without a
  toggle step.
- `+page.svelte` lines ~634–651: `allTabs` — the canonical list of every menu
  item, already exactly matching the spec's requested set:
  `uav-info, mission, control, rc-control, terrain, logbook, radar, airspace,
  video, settings`. `tabs` (derived from `allTabs`) filters some of these
  based on runtime conditions (e.g. `rc-control` only shows when RC is
  available and not telemetry-connected; `control` only when MAVLink
  connected). **Preserve this conditional visibility logic** — don't just
  render all 10 unconditionally.
- Widget components in `src/lib/components/widgets/`: `AHI.svelte`,
  `AltWidget.svelte`, `BatteryWidget.svelte`, `CompassWidget.svelte`,
  `FlightModeWidget.svelte`, `GpsWidget.svelte`, `HomeWidget.svelte`,
  `LiveAglWidget.svelte`, `RawTelemetryWidget.svelte`, `RcLinkWidget.svelte`,
  `SpeedWidget.svelte`, `TerrainRadarWidget.svelte`, `VideoWidget.svelte`.
  All already exist as standalone components — the redesign is a
  **recomposition job**, not a rebuild of telemetry UI.
- `src/lib/components/Map.svelte` and `Map3D.svelte` — existing 2D/3D map
  components, currently rendered full-bleed with widgets floating on top
  (search `<Map` around line ~2455, `map3d-layer` ~2476, `map-video-wrap`
  ~2578 in `+page.svelte`).
- `src/lib/components/WindowResizeBorders.svelte` — handles **Tauri window**
  resize borders (OS-level window resizing), not panel/pane resizing inside
  the app. Don't confuse the two when implementing resizable panels.
- `package.json` — **no split-pane/resizable-panel library is currently
  installed.** Confirm this is still true, then decide: hand-roll a drag
  handle (consistent with the app's existing hand-rolled drag-and-drop in
  `WidgetPanel.svelte`, so may fit the codebase's conventions better) or add
  a small dependency. Prefer hand-rolled unless there's a strong reason not
  to, to stay consistent with existing patterns and avoid an unnecessary new
  dependency in a Tauri desktop app.

## What "resizable" means here (added requirement, not in the original spec doc)

The user asked to "also make it resizable" without further detail. Interpret
this as: **the major regions of the new dashboard layout — map panel,
telemetry strip(s) below the map, and the right-side nav rail — should have
user-draggable resize handles**, in addition to the responsive reflow at
different window sizes already required by the base spec (section 11 below).
Persist the user's chosen split sizes the same way other layout preferences
are persisted in this app (check `settings.ts` / wherever UI scale and panel
config are stored) so it survives a restart. If the existing app has no
precedent for persisting this kind of preference, ask before inventing a new
persistence mechanism — check first whether `PanelConfig` in `settings.ts`
is the natural place to extend, since it already persists widget→panel
assignment.

If, after inspecting the codebase, resizing the map/telemetry split turns out
to conflict with the `WidgetPanel` drag-and-drop system in a way that's hard
to reconcile safely, flag that tradeoff explicitly rather than picking a
solution unilaterally — this is exactly the kind of judgment call worth a
quick confirmation before large structural changes.

## The redesign itself

Everything below is the original spec, unchanged in substance, cross-checked
against the file names Claude Code will actually be editing.

### 1. Visual theme
Dark/charcoal + red accent, Dilaton-inspired (dilaton.ai as visual reference
only — do not copy its text/content). Technical, high-density,
defence/aerospace console aesthetic. Rebrand "KiteGC" → "MAXGCS" everywhere
in the UI (check `+page.svelte` header, `app.html` title, any branding
strings — `static/branding/` may hold logo assets to swap or restyle).

### 2. Layout
- Map (`Map.svelte`/`Map3D.svelte`) moves to the upper-left ~25% of the
  screen, in its own panel, **with no telemetry widgets overlaid on it**.
  Map stays interactive (pan/zoom/layers).
- All telemetry currently floating on the map — MODE, BAT, HOME, SPD, ALT,
  GPS, LINK, AHI, Compass — moves below the map, most likely by repointing
  the existing `WidgetPanel` instance(s) at a new location in the DOM/layout
  rather than rebuilding widget rendering.
- Map controls (3D toggle, zoom, orientation/locate) move off the map into a
  dedicated control cluster near/below the map — locate the current controls
  in `+page.svelte` (near the `<Map` usage, likely conditionally rendered
  alongside `map3d-layer`) and confirm exactly which are overlay buttons
  today before relocating them.
- Right-side nav: convert `NavRail.svelte` from hamburger-toggle to always
  visible individual buttons for every entry in `tabs` (derived from
  `allTabs`), preserving the existing conditional-visibility filters. Keep
  using `onSelectTab`/`activeTab` wiring already in place — this is a visual
  change to `NavRail.svelte`, not new navigation logic.
- Top header: rebrand + restyle, but preserve existing connection controls
  (protocol/MSP/MAVLink selection, serial/device dropdown, baud rate,
  Connect button, relay controls — `RelayPanel.svelte`, `Toolbar.svelte` are
  likely involved here; confirm by reading the header section of
  `+page.svelte`).

### 3. Constraints
- Don't touch flight-control logic, telemetry protocols, MSP/MAVLink
  implementation, serial/Tauri comms, or the map data layer. This is a
  layout/composition/styling change built on existing components and stores.
- Reuse existing components — `WidgetPanel`, `widgetController`,
  `widgetRegistry`, the widget components, `NavRail`, `Map`/`Map3D` — rather
  than rewriting them. Only add new components for layout containers/shells
  that don't already exist (e.g. a new top-level dashboard grid/shell
  component, if `+page.svelte` doesn't already have a clean seam for one).
- No fake/mock data. Existing "N/A"/"—" disconnected-state behavior must be
  preserved.
- Status colors: red = alerts/errors/active nav state, green = healthy,
  amber = warning, neutral = N/A/inactive. Red is the brand accent, not the
  only color.
- Responsive at minimum 1920×1080, 1600×900, 1366×768, and smaller laptop
  sizes, with resizable panel splits added on top per the section above. No
  overlapping panels, no clipped text.

### 4. Routing
`/` becomes the new dashboard. Determine — don't assume — whether splitting
the existing full GCS view out to `/gcs` is safe given SvelteKit routing and
Tauri window/state behavior in this specific app, versus keeping one route
and changing only its internal composition. Pick whichever preserves current
behavior most safely; state the reasoning for whichever is chosen.

## Suggested approach

1. Read `ui-framework.md` and `widgetController.ts`/`settings.ts` fully
   before writing any code — the panel/widget persistence model needs to be
   understood, not guessed at.
2. Get the app running locally (`scripts/dev.sh` or the `justfile` — check
   which is the intended entry point) and confirm the current UI renders, so
   changes can be visually verified step by step rather than shipped blind.
3. Prototype the new layout shell (map region / telemetry region / nav rail
   region, with resize handles) with the *existing* widgets/panels slotted
   in, before doing any visual theming — get the structural change working
   and non-destructive first, theme second.
4. Convert `NavRail` to always-visible buttons.
5. Apply the Dilaton-inspired dark/red theme last, across the new shell and
   the preserved existing panels, checking each existing panel (settings,
   mission, RC control, video, radar, airspace, logbook, terrain, UAV info)
   still opens and functions correctly from the new rail.
6. Test at each target resolution, including dragging the new resize handles
   to confirm panels reflow sanely at the extremes.
