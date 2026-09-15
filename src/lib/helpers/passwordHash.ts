// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

// Salted-hash helpers for the Settings password gate (see stores/auth.ts). This is a soft lock for
// a single-machine desktop app, not a defense against a technical attacker with devtools access —
// Web Crypto's SHA-256 + a random salt is appropriate for that threat model without adding a dependency.

function bufferToHex(buf: ArrayBuffer): string {
  return Array.from(new Uint8Array(buf)).map((b) => b.toString(16).padStart(2, '0')).join('');
}

export function generateSalt(): string {
  const bytes = new Uint8Array(16);
  crypto.getRandomValues(bytes);
  return bufferToHex(bytes.buffer);
}

export async function hashPassword(password: string, salt: string): Promise<string> {
  const data = new TextEncoder().encode(`${salt}:${password}`);
  const digest = await crypto.subtle.digest('SHA-256', data);
  return bufferToHex(digest);
}
