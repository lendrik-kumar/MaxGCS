<!--
  SPDX-License-Identifier: GPL-3.0-or-later
  Copyright (C) 2026 Marc Hoffmann (b14ckyy)
-->

<script lang="ts">
  import { t } from 'svelte-i18n';

  interface Tab {
    id: string;
    label: () => string;
    icon: string;
  }

  let {
    activeTab,
    tabs,
    onSelectTab,
  }: {
    activeTab: string;
    tabs: Tab[];
    onSelectTab: (tabId: string) => void;
  } = $props();
</script>

<div class="nav-rail">
  <div class="tab-buttons">
    {#each tabs as tab}
      {#if tab.id === '__sep__'}
        <div class="tab-sep"></div>
      {:else}
        <button
          class="tab-btn"
          class:active={activeTab === tab.id}
          onclick={() => onSelectTab(tab.id)}
          title={tab.label()}
        >
          <!-- icon is a glyph or an inline SVG string (trusted, app-defined) -->
          <span class="tab-icon">{@html tab.icon}</span>
          <span class="tab-label">{tab.label()}</span>
        </button>
      {/if}
    {/each}
  </div>
</div>

<style>
  .nav-rail {
    position: absolute;
    top: 65px;
    right: 12px;
    display: flex;
    flex-direction: column;
    gap: 0;
    z-index: 100;
  }

  .tab-buttons {
    display: flex;
    flex-direction: column;
    gap: 2px;
  }

  /* Divider between the old panels (top) and the new framework panels (bottom). */
  .tab-sep {
    height: 1px;
    margin: 4px 6px;
    background: var(--mx-red-dim, rgba(224, 48, 44, 0.4));
  }

  .tab-btn {
    display: flex;
    align-items: center;
    gap: 8px;
    width: 42px;
    height: 38px;
    background: var(--mx-panel, rgba(46, 46, 46, 0.92));
    border: 1px solid var(--mx-border, #34363a);
    border-radius: 6px;
    color: var(--mx-text-muted, #a8a8a8);
    font-size: 11px;
    cursor: pointer;
    padding: 0;
    justify-content: center;
    overflow: hidden;
    transition: width 0.3s ease, background-color 0.2s;
    backdrop-filter: blur(8px);
    white-space: nowrap;
  }

  .tab-btn:hover {
    background: var(--mx-red-dim, rgba(224, 48, 44, 0.15));
    color: var(--mx-text, #e0e0e0);
  }

  /* Darker active fill (black 50% + the inherited blur) so the accent border + icon stay
     readable over bright maps; the red border/icon remain the active indicator. */
  .tab-btn.active {
    background: rgba(0, 0, 0, 0.5);
    border-color: var(--mx-red, #e0302c);
    color: var(--mx-red, #e0302c);
  }

  .tab-icon {
    font-size: 16px;
    flex-shrink: 0;
    display: flex;
    align-items: center;
    justify-content: center;
  }

  /* Inline-SVG icons fill the button (≤ ~10% margin to the frame); glyphs keep font-size. */
  .tab-icon :global(svg) {
    width: 32px;
    height: 32px;
    display: block;
  }

  .tab-label {
    display: none;
  }
</style>
