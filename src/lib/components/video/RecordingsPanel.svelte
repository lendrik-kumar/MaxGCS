<!--
  SPDX-License-Identifier: GPL-3.0-or-later
  Copyright (C) 2026 Marc Hoffmann (b14ckyy)
-->

<script lang="ts">
  // Lists the enhanced native-capture recordings written by VideoPanel's Record toggle (see
  // stores/video.ts `setNativeRecording` / `video::recording` on the backend). Read-only for now —
  // no delete/rename here, just "what did I save and where".
  import { t } from 'svelte-i18n';
  import { onMount } from 'svelte';
  import { recordingsList, recordingsLoading, recordingsDir, refreshRecordings } from '$lib/stores/video';
  import PanelShell from '$lib/components/panel/PanelShell.svelte';
  import Button from '$lib/components/panel/Button.svelte';

  onMount(() => {
    void refreshRecordings();
  });

  /** Same formatting as the flight logbook's raw-log size badge (FlightDetail.svelte). */
  function formatBytes(bytes: number): string {
    if (bytes < 1024) return `${bytes} B`;
    if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(0)} KB`;
    return `${(bytes / (1024 * 1024)).toFixed(1)} MB`;
  }
</script>

{#snippet toolbar()}
  <Button variant="standard" onclick={() => void refreshRecordings()} disabled={$recordingsLoading}>
    {$t('recordings.refresh')}
  </Button>
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
      </div>
      {#each $recordingsList as r (r.path)}
        <div class="rec-row" title={r.path}>
          <span class="rec-name">{r.name}</span>
          <span class="rec-size">{formatBytes(r.sizeBytes)}</span>
          <span class="rec-date">{new Date(r.modifiedMs).toLocaleString()}</span>
        </div>
      {/each}
    {/if}
  </div>
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
    grid-template-columns: 1fr 64px 130px;
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
</style>
