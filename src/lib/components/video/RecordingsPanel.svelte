<!--
  SPDX-License-Identifier: GPL-3.0-or-later
  Copyright (C) 2026 Marc Hoffmann (b14ckyy)
-->

<script lang="ts">
  // Lists the enhanced native-capture recordings written by VideoPanel's Record toggle (see
  // stores/video.ts `setNativeRecording` / `video::recording` on the backend), and exports one to a
  // shareable .mp4 (`video::export`). No delete/rename — a recording is the only copy of its
  // footage, so this panel never removes one.
  import { t } from 'svelte-i18n';
  import { onMount } from 'svelte';
  import { save } from '@tauri-apps/plugin-dialog';
  import {
    recordingsList, recordingsLoading, recordingsDir, refreshRecordings,
    exportRecording, cancelExport, exportSuggestedName, exportProgress,
    type ExportPreset, type RecordingInfo,
  } from '$lib/stores/video';
  import PanelShell from '$lib/components/panel/PanelShell.svelte';
  import Button from '$lib/components/panel/Button.svelte';
  import SegmentedToggle from '$lib/components/panel/SegmentedToggle.svelte';

  onMount(() => {
    void refreshRecordings();
  });

  // The preset applies to whichever recording is exported next; it is a property of the export,
  // not of a row, so it lives in the toolbar rather than being repeated per row.
  let preset = $state<ExportPreset>('original');
  /** Path of the recording currently exporting — marks its row and disables the other buttons. */
  let exporting = $state<string | null>(null);
  let exportError = $state<string | null>(null);

  const presetOptions = $derived([
    { value: 'original', label: $t('recordings.presetOriginal') },
    { value: 'cleaned', label: $t('recordings.presetCleaned') },
    { value: 'upscale', label: $t('recordings.presetUpscale') },
  ]);

  async function doExport(r: RecordingInfo) {
    exportError = null;
    try {
      const defaultName = await exportSuggestedName(r.path, preset);
      const outputPath = await save({
        filters: [{ name: 'MP4 Video', extensions: ['mp4'] }],
        defaultPath: defaultName,
      });
      if (!outputPath) return;
      exporting = r.path;
      await exportRecording(r.path, outputPath, preset);
    } catch (e) {
      // A cancelled export reports itself as an error from the backend; it is not worth surfacing
      // as one, since the user is the one who cancelled it.
      const msg = e instanceof Error ? e.message : String(e);
      if (!msg.includes('cancelled')) exportError = msg;
    } finally {
      exporting = null;
    }
  }

  /** Same formatting as the flight logbook's raw-log size badge (FlightDetail.svelte). */
  function formatBytes(bytes: number): string {
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(0)} KB`;
    return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
  }
</script>

{#snippet toolbar()}
  <Button variant="standard" onclick={() => void refreshRecordings()} disabled={$recordingsLoading || !!exporting}>
    {$t('recordings.refresh')}
  </Button>
  <span class="preset-label">{$t('recordings.exportAs')}</span>
  <SegmentedToggle
    options={presetOptions}
    value={preset}
    size="sm"
    disabled={!!exporting}
    onchange={(v) => (preset = v as ExportPreset)}
  />
{/snippet}

{#snippet body()}
  <div class="rec-list">
    {#if $recordingsList.length === 0}
      <p class="hint">{$t('recordings.empty')}</p>
    {:else}
      <div class="rec-header">
        <span>{$t('recordings.name')}</span>
        <span>{$t('recordings.size')}</span>
        <span>{$t('recordings.date')}</span>
        <span></span>
      </div>
      {#each $recordingsList as r (r.path)}
        <div class="rec-row" class:busy={exporting === r.path} title={r.path}>
          <span class="rec-name">{r.name}</span>
          <span class="rec-size">{formatBytes(r.sizeBytes)}</span>
          <span class="rec-date">{new Date(r.modifiedMs).toLocaleString()}</span>
          {#if exporting === r.path}
            <Button variant="danger" size="sm" onclick={() => void cancelExport()}>
              {$t('dialog.cancel')}
            </Button>
          {:else}
            <Button
              variant="data"
              size="sm"
              icon="export"
              disabled={!!exporting}
              onclick={() => void doExport(r)}
            >{$t('recordings.export')}</Button>
          {/if}
        </div>
        {#if exporting === r.path && $exportProgress}
          <div class="rec-progress">
            <div class="bar"><div class="fill" style:width={`${$exportProgress.pct}%`}></div></div>
            <span class="pct">{$exportProgress.pct}%</span>
          </div>
        {/if}
      {/each}
    {/if}
  </div>
  {#if exportError}
    <p class="err">{exportError}</p>
  {/if}
  {#if $recordingsDir}
    <p class="hint folder-hint">{$t('recordings.folderHint', { values: { path: $recordingsDir } })}</p>
  {/if}
{/snippet}

<PanelShell variant="compact" title={$t('recordings.title')} {toolbar} {body} />

<style>
  .rec-list {
    display: flex;
    flex-direction: column;
    gap: 2px;
  }

  .rec-header,
  .rec-row {
    display: grid;
    grid-template-columns: 1fr 64px 130px auto;
    gap: 8px;
    align-items: center;
    padding: 4px 6px;
    font-size: 12px;
  }

  .rec-header {
    color: var(--mx-text-muted, #a8a8a8);
    font-weight: 600;
    text-transform: uppercase;
    font-size: 10px;
    letter-spacing: 0.03em;
    border-bottom: 1px solid #444;
  }

  .rec-row {
    border-radius: 4px;
  }

  .rec-row:hover {
    background: rgba(255, 255, 255, 0.05);
  }

  .rec-name {
    overflow: hidden;
    text-overflow: ellipsis;
    white-space: nowrap;
  }

  .rec-size,
  .rec-date {
    color: var(--mx-text-muted, #a8a8a8);
    white-space: nowrap;
  }

  .hint {
    font-size: 12px;
    color: var(--mx-text-muted, #a8a8a8);
    margin: 6px 2px;
  }

  .folder-hint {
    word-break: break-all;
  }

  .preset-label {
    font-size: 10px;
    font-weight: 600;
    text-transform: uppercase;
    letter-spacing: 0.03em;
    color: var(--mx-text-muted, #a8a8a8);
    margin-left: 10px;
    align-self: center;
  }

  .rec-row.busy {
    background: rgba(37, 144, 200, 0.12);
  }

  .rec-progress {
    display: flex;
    align-items: center;
    gap: 8px;
    padding: 0 6px 6px;
  }

  .bar {
    flex: 1;
    height: 4px;
    border-radius: 2px;
    background: #444;
    overflow: hidden;
  }

  .fill {
    height: 100%;
    background: #2590c8;
    transition: width 0.2s linear;
  }

  .pct {
    font-size: 11px;
    color: var(--mx-text-muted, #a8a8a8);
    min-width: 32px;
    text-align: right;
  }

  .err {
    font-size: 12px;
    color: #e0302c;
    margin: 6px 2px;
    word-break: break-word;
  }
</style>
