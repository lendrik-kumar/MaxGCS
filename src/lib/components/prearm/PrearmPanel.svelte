<!--
  SPDX-License-Identifier: GPL-3.0-or-later
-->

<script lang="ts">
  // Pre-arm panel: a button that asks the flight controller "would you arm right now?" and, if not, lists every
  // blocker with the FC's exact message, a plain title, WHY it happens (with the live readings that show it) and
  // HOW to fix it. Works for MAVLink (on-demand MAV_CMD_RUN_PREARM_CHECKS + the FC's PreArm messages) and INAV
  // (its arming-disabled flags). Arms nothing. Compact PanelShell like the Video panel.
  import { t } from 'svelte-i18n';
  import PanelShell from '$lib/components/panel/PanelShell.svelte';
  import Button from '$lib/components/panel/Button.svelte';
  import { telemetry } from '$lib/stores/telemetry';
  import { connection } from '$lib/stores/connection';
  import { autopilotSystem } from '$lib/stores/autopilotContext';
  import { prearmReport, prearmRunning, runPrearmCheck, type PrearmIssue } from '$lib/stores/prearm';
  import { isArmed } from '$lib/helpers/arming';

  const connected = $derived($connection.status === 'connected' && $telemetry.lastUpdate > 0);
  const isInav = $derived($autopilotSystem === 'inav');
  const report = $derived($prearmReport);

  // Live readings: the numbers behind the explanations (is the GPS really without a fix? is the battery low?).
  const gpsText = $derived(
    $telemetry.fixType === 0 ? $t('gps.noFix') : $telemetry.fixType === 1 ? $t('gps.fix2d') : $telemetry.fixType === 2 ? $t('gps.fix3d') : $t('gps.fix3dDgps'),
  );
  type Tone = 'ok' | 'bad' | 'off';
  const sensors = $derived<{ key: string; label: string; tone: Tone }[]>(
    [
      ['gyro', $t('sensors.gyro'), $telemetry.sensorGyro],
      ['acc', $t('sensors.acc'), $telemetry.sensorAcc],
      ['mag', $t('sensors.mag'), $telemetry.sensorMag],
      ['baro', $t('sensors.baro'), $telemetry.sensorBaro],
      ['gps', $t('sensors.gps'), $telemetry.sensorGps],
    ].map(([key, label, st]) => ({ key: key as string, label: label as string, tone: (st === 0 ? 'off' : st === 1 ? 'ok' : 'bad') as Tone })),
  );

  const verdictKey = $derived(report ? report.level : 'idle');
  const verdictText = $derived($t(`prearm.verdict.${verdictKey}`));
  const verdictDetail = $derived.by(() => {
    if (!report) return $t('prearm.intro');
    switch (report.level) {
      case 'running': return $t('prearm.detail.running');
      case 'ready': return $t('prearm.detail.ready');
      case 'blocked': return $t('prearm.detail.blocked', { values: { count: report.issues.length || 1 } });
      case 'armed': return $t('prearm.detail.armed');
      case 'unsupported': return $t('prearm.detail.unsupported');
      default: return noteText(report.note);
    }
  });

  function noteText(note: string | null): string {
    if (!note) return '';
    if (note === 'not-connected') return $t('prearm.notConnected');
    return note;
  }

  function when(at: number): string {
    return new Date(at).toLocaleTimeString();
  }

  // Explanation texts: MAVLink rules are keyed by the matched rule; INAV by the arming-flag reason.
  const titleOf = (i: PrearmIssue) => (i.source === 'inav' ? $t(`arming.reasons.${i.key}`) : $t(`prearm.rules.${i.key}.title`));
  const whyOf = (i: PrearmIssue) =>
    i.source === 'inav' ? $t(`prearm.inav.${i.key}.why`, { default: $t('prearm.inavGeneric.why') }) : $t(`prearm.rules.${i.key}.why`);
  const fixOf = (i: PrearmIssue) =>
    i.source === 'inav' ? $t(`prearm.inav.${i.key}.fix`, { default: $t('prearm.inavGeneric.fix') }) : $t(`prearm.rules.${i.key}.fix`);

  /** The live reading most relevant to this problem, so "why" is backed by a number. */
  function evidence(i: PrearmIssue): string | null {
    const k = i.key;
    if (k === 'gps' || k === 'home')
      return `${$t('prearm.live.gps')}: ${gpsText} · ${$telemetry.numSat} ${$t('prearm.live.satellites')}` + ($telemetry.gpsHdop ? ` · HDOP ${$telemetry.gpsHdop.toFixed(1)}` : '');
    if (k === 'battery') return `${$t('prearm.live.battery')}: ${$telemetry.voltage > 0 ? $telemetry.voltage.toFixed(2) + ' V' : '—'}`;
    if (k === 'compass') return `${$t('sensors.mag')}: ${$t(`prearm.live.${sensors[2].tone}`)}`;
    if (k === 'gyro') return `${$t('sensors.gyro')}: ${$t(`prearm.live.${sensors[0].tone}`)}`;
    if (k === 'accel') return `${$t('sensors.acc')}: ${$t(`prearm.live.${sensors[1].tone}`)}`;
    if (k === 'baro') return `${$t('sensors.baro')}: ${$t(`prearm.live.${sensors[3].tone}`)}`;
    return null;
  }
</script>

{#snippet headerActions()}
  <Button variant="data" disabled={!connected || $prearmRunning} onclick={runPrearmCheck}>
    {$prearmRunning ? $t('prearm.running') : $t('prearm.run')}
  </Button>
{/snippet}

{#snippet body()}
  <div class="pa-body">
    {#if !connected}
      <div class="pa-verdict idle">
        <div class="pa-verdict-title">{$t('prearm.notConnected')}</div>
      </div>
    {:else}
      <div class="pa-verdict {verdictKey}">
        <div class="pa-verdict-title">{verdictText}</div>
        <div class="pa-verdict-detail">{verdictDetail}</div>
        {#if report && report.level !== 'running'}
          <div class="pa-when">{$t('prearm.lastChecked', { values: { time: when(report.at) } })}</div>
        {/if}
      </div>

      <dl class="pa-live">
        <dt>{$t('prearm.live.armed')}</dt>
        <dd>{isArmed($telemetry.armingFlags) ? $t('arming.armed') : $t('arming.disarmed')}</dd>
        <dt>{$t('prearm.live.mode')}</dt>
        <dd>{$telemetry.flightMode.primary || '—'}</dd>
        <dt>{$t('prearm.live.gps')}</dt>
        <dd>{gpsText} · {$telemetry.numSat} {$t('prearm.live.satellites')}</dd>
        <dt>{$t('prearm.live.battery')}</dt>
        <dd>{$telemetry.voltage > 0 ? `${$telemetry.voltage.toFixed(2)} V` : '—'}</dd>
      </dl>
      <div class="pa-sensors">
        {#each sensors as s (s.key)}
          <span class="pa-sensor {s.tone}" title={$t(`prearm.live.${s.tone}`)}>{s.label}</span>
        {/each}
      </div>

      {#if report && report.issues.length}
        <div class="pa-issues">
          {#each report.issues as issue, n (issue.source + issue.raw + n)}
            {@const ev = evidence(issue)}
            <section class="pa-issue">
              <h4>{titleOf(issue)}</h4>
              {#if issue.source === 'fc'}
                <div class="pa-raw"><span>{$t('prearm.fcSaid')}</span><code>{issue.raw}</code></div>
              {/if}
              <p><b>{$t('prearm.why')}</b> {whyOf(issue)}</p>
              {#if ev}<p class="pa-ev">{ev}</p>{/if}
              <p><b>{$t('prearm.fix')}</b> {fixOf(issue)}</p>
            </section>
          {/each}
        </div>
      {:else if report && report.level === 'blocked'}
        <div class="pa-issue">
          <h4>{$t('prearm.rules.generic.title')}</h4>
          <p>{$t('prearm.noMessages')}</p>
        </div>
      {/if}

      {#if report && report.note && report.level !== 'running' && report.level !== 'ready'}
        <div class="pa-note">{noteText(report.note)}</div>
      {/if}
    {/if}
  </div>
{/snippet}

{#snippet footer()}
  <div class="pa-footer">{$t('prearm.footer')}</div>
{/snippet}

<PanelShell variant="compact" title={$t('prearm.title')} {headerActions} {body} {footer} />

<style>
  .pa-body { display: flex; flex-direction: column; gap: 10px; }
  .pa-verdict { border-radius: 6px; padding: 9px 11px; border: 1px solid rgba(255, 255, 255, 0.12); background: rgba(255, 255, 255, 0.04); }
  .pa-verdict-title { font-size: 13px; font-weight: 700; letter-spacing: 0.02em; }
  .pa-verdict-detail { font-size: 11.5px; color: #c4cbd0; margin-top: 3px; }
  .pa-when { font-size: 10.5px; color: #8a949a; margin-top: 4px; }
  .pa-verdict.ready { border-color: rgba(79, 196, 122, 0.55); background: rgba(79, 196, 122, 0.12); }
  .pa-verdict.ready .pa-verdict-title { color: #6fe09a; }
  .pa-verdict.blocked, .pa-verdict.error { border-color: rgba(212, 0, 0, 0.6); background: rgba(212, 0, 0, 0.14); }
  .pa-verdict.blocked .pa-verdict-title, .pa-verdict.error .pa-verdict-title { color: #ff6a6a; }
  .pa-verdict.armed { border-color: rgba(224, 165, 60, 0.6); background: rgba(224, 165, 60, 0.13); }
  .pa-verdict.armed .pa-verdict-title { color: #f0b84e; }
  .pa-verdict.unsupported, .pa-verdict.running { border-color: rgba(74, 163, 223, 0.55); background: rgba(74, 163, 223, 0.12); }
  .pa-live { display: grid; grid-template-columns: auto 1fr; gap: 2px 12px; font-size: 12px; margin: 0; }
  .pa-live dt { color: #8a949a; }
  .pa-live dd { margin: 0; text-align: right; font-variant-numeric: tabular-nums; }
  .pa-sensors { display: flex; gap: 5px; flex-wrap: wrap; }
  .pa-sensor { font-size: 10.5px; font-weight: 600; padding: 2px 7px; border-radius: 3px; border: 1px solid transparent; }
  .pa-sensor.ok { background: rgba(79, 196, 122, 0.16); color: #6fe09a; border-color: rgba(79, 196, 122, 0.4); }
  .pa-sensor.bad { background: rgba(212, 0, 0, 0.18); color: #ff6a6a; border-color: rgba(212, 0, 0, 0.5); }
  .pa-sensor.off { background: rgba(255, 255, 255, 0.05); color: #8a949a; border-color: rgba(255, 255, 255, 0.12); }
  .pa-issues { display: flex; flex-direction: column; gap: 8px; }
  .pa-issue { border: 1px solid rgba(212, 0, 0, 0.35); border-left-width: 3px; border-radius: 5px; padding: 8px 10px; background: rgba(0, 0, 0, 0.18); }
  .pa-issue h4 { margin: 0 0 5px; font-size: 12.5px; color: #ff8a8a; }
  .pa-issue p { margin: 4px 0 0; font-size: 11.5px; line-height: 1.4; color: #d3d9dd; }
  .pa-issue b { color: #ffffff; }
  .pa-ev { color: #9ad0e8 !important; font-variant-numeric: tabular-nums; }
  .pa-raw { display: flex; flex-direction: column; gap: 2px; font-size: 10.5px; color: #8a949a; }
  .pa-raw code { color: #e8d9a8; font-size: 11px; background: #1b1b1b; border-radius: 3px; padding: 2px 6px; word-break: break-word; }
  .pa-note { font-size: 11px; color: #e0a53c; }
  .pa-footer { font-size: 10.5px; color: #8a949a; line-height: 1.35; }
</style>
