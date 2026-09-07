<!--
  SPDX-License-Identifier: GPL-3.0-or-later
  Copyright (C) 2026 Marc Hoffmann (b14ckyy)
-->

<!-- MissionLayer.svelte — thin switcher
     Delegates to INAV or ArduPilot map layer based on active autopilot system.
     Usage: <MissionLayer {map} />
-->
<script lang="ts">
  import { onDestroy } from 'svelte';
  import { get } from 'svelte/store';
  import { autopilotSystem, type AutopilotSystem } from '$lib/stores/autopilotContext';
  import InavMissionLayer from './InavMissionLayer.svelte';
  import ArduMissionLayer from './ArduMissionLayer.svelte';
  import type L from 'leaflet';
  import { invoke } from '@tauri-apps/api/core';

  interface Props { map: L.Map; }
  let { map }: Props = $props();

  let currentSystem = $state<AutopilotSystem>(get(autopilotSystem));
  const unsub = autopilotSystem.subscribe(s => { currentSystem = s; });
  onDestroy(() => unsub());

  // TEMPORARY DIAGNOSTIC (remove alongside the InavMissionLayer traces) — confirms which layer
  // component this dispatcher mounts, every time it switches.
  $effect(() => {
    void invoke('log_frontend', { level: 'warn', area: 'inav-wp-debug', message: `MissionLayer: currentSystem=${JSON.stringify(currentSystem)} -> rendering ${currentSystem === 'inav' ? 'InavMissionLayer' : 'ArduMissionLayer'}` }).catch(() => {});
  });
</script>

{#if currentSystem === 'inav'}
  <InavMissionLayer {map} />
{:else}
  <ArduMissionLayer {map} />
{/if}
