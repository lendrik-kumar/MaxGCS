// SPDX-License-Identifier: GPL-3.0-or-later

// On-demand pre-arm check: ask the flight controller whether it would arm right now, and gather WHY not.
//
//  • MAVLink (ArduPilot / PX4): send MAV_CMD_RUN_PREARM_CHECKS (backend command `mav_run_prearm_checks`). The
//    FC answers "passed" / "failed" and reports each failing check as a "PreArm: …" STATUSTEXT, so we listen
//    to `mavlink-statustext` from just before the request until a short grace period after the answer.
//  • INAV (MSP): there is nothing to ask — the FC publishes its blockers continuously as armingFlags, so the
//    result is computed instantly from the live telemetry.
//
// Nothing here arms the vehicle; it only reads.

import { writable, get } from 'svelte/store';
import { invoke } from '@tauri-apps/api/core';
import { listen, type UnlistenFn } from '@tauri-apps/api/event';
import { telemetry } from './telemetry';
import { autopilotSystem } from './autopilotContext';
import { connection } from './connection';
import { inavDisableReasonKeys, isArmed } from '$lib/helpers/arming';
import { isPrearmMessage, prearmRuleKey, stripPrearmPrefix } from '$lib/helpers/prearmExplain';

export type PrearmLevel = 'idle' | 'running' | 'ready' | 'blocked' | 'armed' | 'unsupported' | 'error';

export interface PrearmIssue {
  /** `fc` = a message the FC sent (MAVLink); `inav` = an INAV ARMING_DISABLED_* flag. */
  source: 'fc' | 'inav';
  /** The FC's exact message (MAVLink) or the INAV flag's label key suffix. */
  raw: string;
  /** MAVLink: explanation rule key (`prearm.rules.<key>`). INAV: the arming reason key (`prearm.inav.<key>`). */
  key: string;
  /** MAV_SEVERITY of the message (INAV: 2). */
  severity: number;
}

export interface PrearmReport {
  level: PrearmLevel;
  at: number; // epoch ms
  issues: PrearmIssue[];
  /** MAVLink: what the FC answered to the request (null if it never did / not MAVLink). */
  fcAnswer: 'passed' | 'failed' | null;
  /** A human-readable problem with the check itself (unsupported, timed out, armed, not connected…). */
  note: string | null;
}

export const prearmReport = writable<PrearmReport | null>(null);
export const prearmRunning = writable(false);

const GRACE_MS = 1500; // ArduPilot sends the PreArm lines around its ack; give stragglers a moment

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

function finish(report: Omit<PrearmReport, 'at'>): void {
  prearmReport.set({ ...report, at: Date.now() });
}

/** INAV: the answer is already in the telemetry. */
function checkInav(): void {
  const t = get(telemetry);
  if (isArmed(t.armingFlags)) return finish({ level: 'armed', issues: [], fcAnswer: null, note: null });
  const keys = inavDisableReasonKeys(t.armingFlags);
  finish({
    level: keys.length ? 'blocked' : 'ready',
    issues: keys.map((k) => ({ source: 'inav' as const, raw: k, key: k, severity: 2 })),
    fcAnswer: null,
    note: null,
  });
}

/** MAVLink: run the FC's checks and collect its messages. */
async function checkMavlink(): Promise<void> {
  const t0 = get(telemetry);
  if (isArmed(t0.armingFlags)) {
    return finish({ level: 'armed', issues: [], fcAnswer: null, note: null });
  }

  const seen = new Map<string, PrearmIssue>();
  let unlisten: UnlistenFn | null = null;
  try {
    unlisten = await listen<{ severity: number; text: string }>('mavlink-statustext', (e) => {
      const { severity, text } = e.payload;
      // Arming blockers carry a PreArm:/Arm: prefix; also keep any error-level (<= 4) line during the check.
      if (!isPrearmMessage(text) && severity > 4) return;
      const raw = stripPrearmPrefix(text);
      if (!seen.has(raw)) seen.set(raw, { source: 'fc', raw, key: prearmRuleKey(text), severity });
    });

    let answer: 'passed' | 'failed' | null = null;
    let note: string | null = null;
    let level: PrearmLevel | null = null;
    try {
      answer = (await invoke<string>('mav_run_prearm_checks')) as 'passed' | 'failed';
    } catch (err) {
      const msg = String(err);
      note = msg;
      level = /cannot run pre-arm|not supported|unsupported/i.test(msg) ? 'unsupported' : /armed/i.test(msg) ? 'armed' : 'error';
    }
    await sleep(GRACE_MS);

    const issues = [...seen.values()];
    if (level === 'armed') return finish({ level: 'armed', issues: [], fcAnswer: null, note: null });
    if (level === 'unsupported') {
      // Older firmware: no on-demand check. Fall back to whatever the FC has said recently + its health bit.
      const blocked = issues.length > 0 || get(telemetry).prearmHealthy === 2;
      return finish({ level: blocked ? 'blocked' : 'unsupported', issues, fcAnswer: null, note });
    }
    if (level === 'error') return finish({ level: 'error', issues, fcAnswer: null, note });

    // The FC ran the checks. FAILED, any PreArm message, or its own "blocked" health bit => not ready.
    const blocked = answer === 'failed' || issues.length > 0 || get(telemetry).prearmHealthy === 2;
    finish({ level: blocked ? 'blocked' : 'ready', issues, fcAnswer: answer, note: null });
  } finally {
    unlisten?.();
  }
}

/** Run the pre-arm check now. Safe to call repeatedly; ignored while one is running. */
export async function runPrearmCheck(): Promise<void> {
  if (get(prearmRunning)) return;
  if (get(connection).status !== 'connected') {
    return finish({ level: 'error', issues: [], fcAnswer: null, note: 'not-connected' });
  }
  prearmRunning.set(true);
  prearmReport.update((r) => (r ? { ...r, level: 'running' } : { level: 'running', at: Date.now(), issues: [], fcAnswer: null, note: null }));
  try {
    if (get(autopilotSystem) === 'inav') checkInav();
    else await checkMavlink();
  } catch (err) {
    finish({ level: 'error', issues: [], fcAnswer: null, note: String(err) });
  } finally {
    prearmRunning.set(false);
  }
}
