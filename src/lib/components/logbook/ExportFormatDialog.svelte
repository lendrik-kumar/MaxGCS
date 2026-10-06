<!--
  SPDX-License-Identifier: GPL-3.0-or-later
  Copyright (C) 2026 Marc Hoffmann (b14ckyy)
-->

<script lang="ts" module>
  /** One selectable export target. `ext` is what actually picks the exporter — the backend
   *  dispatches on the output file's extension (see flightlog/track_export.rs). */
  export interface ExportFormat {
    ext: string;
    /** Format name as shown in the list and in the native save dialog's filter. */
    label: string;
    /** i18n key for the one-line explanation under the label. */
    descKey: string;
  }

  /** Readable logs — written to be opened and read, every recorded sample kept. */
  export const READABLE_FORMATS: ExportFormat[] = [
    { ext: 'txt',  label: 'TXT',      descKey: 'logbook.fmtTxt' },
    { ext: 'md',   label: 'Markdown', descKey: 'logbook.fmtMd' },
    { ext: 'json', label: 'JSON',     descKey: 'logbook.fmtJson' },
  ];

  /** Geometry & spreadsheet — the flight's path for other tools, GPS-filtered and simplified. */
  export const TRACK_FORMATS: ExportFormat[] = [
    { ext: 'kmz', label: 'KMZ', descKey: 'logbook.fmtKmz' },
    { ext: 'kml', label: 'KML', descKey: 'logbook.fmtKml' },
    { ext: 'gpx', label: 'GPX', descKey: 'logbook.fmtGpx' },
    { ext: 'csv', label: 'CSV', descKey: 'logbook.fmtCsv' },
  ];

  export const ALL_FORMATS: ExportFormat[] = [...READABLE_FORMATS, ...TRACK_FORMATS];
</script>

<script lang="ts">
  import { t } from 'svelte-i18n';
  import Button from '$lib/components/panel/Button.svelte';

  // Last pick is remembered for the session: exporting the same format for several flights in a
  // row is the common case, so the dialog reopens on the previous choice rather than resetting.
  let selected = $state<string>('txt');
  let open = $state(false);
  let resolver: ((value: ExportFormat | null) => void) | null = null;

  /** Show the picker; resolves to the chosen format, or null if cancelled. */
  export function show(): Promise<ExportFormat | null> {
    open = true;
    return new Promise<ExportFormat | null>((resolve) => {
      resolver = resolve;
    });
  }

  function close(value: ExportFormat | null) {
    open = false;
    if (resolver) {
      resolver(value);
      resolver = null;
    }
  }

  function confirm() {
    close(ALL_FORMATS.find((f) => f.ext === selected) ?? null);
  }

  function onKeydown(e: KeyboardEvent) {
    if (e.key === 'Escape') {
      close(null);
      return;
    }
    if (e.key === 'Enter') {
      confirm();
      return;
    }
    // Arrow keys walk the combined list, so the two groups behave as one column.
    if (e.key === 'ArrowDown' || e.key === 'ArrowUp') {
      e.preventDefault();
      const i = ALL_FORMATS.findIndex((f) => f.ext === selected);
      const next = e.key === 'ArrowDown' ? i + 1 : i - 1;
      if (next >= 0 && next < ALL_FORMATS.length) selected = ALL_FORMATS[next].ext;
    }
  }
</script>

<svelte:window onkeydown={open ? onKeydown : undefined} />

{#if open}
  <!-- svelte-ignore a11y_no_static_element_interactions -->
  <!-- svelte-ignore a11y_click_events_have_key_events -->
  <div class="dialog-backdrop" onclick={() => close(null)}>
    <!-- svelte-ignore a11y_no_static_element_interactions -->
    <!-- svelte-ignore a11y_click_events_have_key_events -->
    <div class="dialog-box" onclick={(e) => e.stopPropagation()}>
      <div class="dialog-title">{$t('logbook.exportFormatTitle')}</div>

      <fieldset class="group">
        <legend>{$t('logbook.fmtGroupReadable')}</legend>
        {#each READABLE_FORMATS as f}
          <label class="row" class:sel={selected === f.ext}>
            <input type="radio" bind:group={selected} value={f.ext} />
            <span class="row-text">
              <span class="row-label">{f.label}<span class="row-ext">.{f.ext}</span></span>
              <span class="row-desc">{$t(f.descKey)}</span>
            </span>
          </label>
        {/each}
      </fieldset>

      <fieldset class="group">
        <legend>{$t('logbook.fmtGroupTrack')}</legend>
        {#each TRACK_FORMATS as f}
          <label class="row" class:sel={selected === f.ext}>
            <input type="radio" bind:group={selected} value={f.ext} />
            <span class="row-text">
              <span class="row-label">{f.label}<span class="row-ext">.{f.ext}</span></span>
              <span class="row-desc">{$t(f.descKey)}</span>
            </span>
          </label>
        {/each}
      </fieldset>

      <div class="dialog-buttons">
        <Button variant="compact" onclick={() => close(null)}>{$t('dialog.cancel')}</Button>
        <Button variant="data" icon="export" onclick={confirm}>{$t('logbook.exportTrack')}</Button>
      </div>
    </div>
  </div>
{/if}

<style>
  .dialog-backdrop {
    position: fixed;
    inset: 0;
    z-index: 9999;
    background: rgba(0, 0, 0, 0.55);
    display: flex;
    align-items: center;
    justify-content: center;
  }

  .dialog-box {
    background: #2e2e2e;
    border: 1px solid var(--mx-red-dim, rgba(224, 48, 44, 0.45));
    border-radius: 8px;
    box-shadow: 0 8px 32px rgba(0, 0, 0, 0.5);
    padding: 20px 24px 16px;
    min-width: 400px;
    max-width: 520px;
    max-height: 86vh;
    overflow-y: auto;
  }

  .dialog-title {
    font-size: 14px;
    font-weight: 700;
    color: #e0e0e0;
    margin-bottom: 14px;
  }

  .group {
    border: none;
    border-top: 1px solid #4a4a4a;
    margin: 0 0 12px;
    padding: 6px 0 0;
  }

  .group legend {
    font-size: 10px;
    font-weight: 700;
    letter-spacing: 0.6px;
    text-transform: uppercase;
    color: #8c8c8c;
    padding: 0 6px 0 0;
  }

  .row {
    display: flex;
    align-items: flex-start;
    gap: 9px;
    padding: 6px 8px;
    border-radius: 4px;
    cursor: pointer;
    border: 1px solid transparent;
  }

  .row:hover {
    background: #383838;
  }

  .row.sel {
    background: #333d44;
    border-color: #2590c8;
  }

  .row input {
    accent-color: var(--mx-red, #e0302c);
    margin-top: 2px;
    flex: none;
  }

  .row-text {
    display: flex;
    flex-direction: column;
    gap: 2px;
    min-width: 0;
  }

  .row-label {
    font-size: 12px;
    font-weight: 600;
    color: #e0e0e0;
  }

  .row-ext {
    font-size: 11px;
    font-weight: 400;
    color: #8c8c8c;
    margin-left: 6px;
  }

  .row-desc {
    font-size: 11px;
    color: #a8a8a8;
    line-height: 1.4;
  }

  .dialog-buttons {
    display: flex;
    justify-content: flex-end;
    gap: 8px;
    margin-top: 4px;
  }
</style>
