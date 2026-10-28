// SPDX-License-Identifier: GPL-3.0-or-later

// Turns a raw flight-controller pre-arm message into a plain explanation key.
//
// ArduPilot (and PX4) report why they refuse to arm as short STATUSTEXT lines such as
// "PreArm: Compass not calibrated". Those are accurate but terse. The Pre-arm panel shows the FC's exact
// message AND a plain-language title, the reason it happens, and what to do — looked up here by keyword.
// The texts live in the locale files under `prearm.rules.<key>.{title,why,fix}`; a message that matches no
// rule gets `generic`, so an unknown message is still shown verbatim with general advice.
//
// Rules are tried in order and the first match wins, so the more specific ones come first
// (e.g. "Throttle (RC3) is not configured" must hit `throttle` before the broader `rc`).

export interface PrearmRule {
  key: string;
  test: RegExp;
}

export const PREARM_RULES: PrearmRule[] = [
  { key: 'home', test: /\bhome\b/i },
  { key: 'gps', test: /\bgps\b|3d fix|position estimate|need position|satellite|hdop|gnss|glonass/i },
  { key: 'compass', test: /compass|magnet|\bmag\b|mag field/i },
  { key: 'gyro', test: /gyro/i },
  { key: 'accel', test: /accel|not level|\blevel\b/i },
  { key: 'baro', test: /baro|altitude disparity|\bpressure\b/i },
  { key: 'ekf', test: /\bekf|ahrs|attitude|estimator|inertial|\bimu\b|\byaw\b/i },
  { key: 'battery', test: /batter|voltage|\bmah\b|capacity/i },
  { key: 'throttle', test: /throttle/i },
  { key: 'rc', test: /\brc\b|radio|transmitter|pilot input|receiver|rssi|\bchannel/i },
  { key: 'safety', test: /safety/i },
  { key: 'fence', test: /fence/i },
  { key: 'mode', test: /\bmode\b/i },
  { key: 'motors', test: /motor|\bframe\b|\besc\b|servo|\bpwm\b|output|actuator|rotor|propeller/i },
  { key: 'logging', test: /logging|sd card|storage|\blog\b/i },
  { key: 'terrain', test: /terrain/i },
  { key: 'proximity', test: /proximity|rangefinder|lidar|obstacle/i },
  { key: 'internal', test: /internal error|memory|stack|overrun|firmware|clock|watchdog/i },
  { key: 'params', test: /param|setting|invalid|config|check [a-z_]+/i },
];

/** ArduPilot: "PreArm: …" (refusal to arm) and "Arm: …" (a check that fails at arm time); PX4: "Preflight Fail: …"
 *  / "Arming denied: …". */
export const PREARM_PREFIX = /^(pre[\s-]?arm|preflight fail|arming denied|arm)[:\s]+/i;

/** True for a message that reports an arming blocker. */
export function isPrearmMessage(text: string): boolean {
  return PREARM_PREFIX.test(text.trim());
}

/** The message without its "PreArm:" style prefix. */
export function stripPrearmPrefix(text: string): string {
  const clean = text.trim();
  return clean.replace(PREARM_PREFIX, '').trim() || clean;
}

/** Which explanation rule applies to this FC message (`generic` when none does). */
export function prearmRuleKey(text: string): string {
  const body = stripPrearmPrefix(text);
  return PREARM_RULES.find((r) => r.test.test(body))?.key ?? 'generic';
}
