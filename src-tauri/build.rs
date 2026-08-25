// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Marc Hoffmann (b14ckyy)

use std::process::Command;

fn main() {
    // Git short hash → baked in as KITE_GIT_HASH for the log session header / About (best-effort:
    // "unknown" when git isn't available, e.g. a source tarball build). Rebuild when HEAD moves.
    let git_hash = Command::new("git")
        .args(["rev-parse", "--short", "HEAD"])
        .output()
        .ok()
        .filter(|o| o.status.success())
        .and_then(|o| String::from_utf8(o.stdout).ok())
        .map(|s| s.trim().to_string())
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".into());
    println!("cargo:rustc-env=KITE_GIT_HASH={git_hash}");
    println!("cargo:rerun-if-changed=../.git/HEAD");

    // App version → baked in as KITE_APP_VERSION for the log session header. The crate version in
    // Cargo.toml is NOT the app version (deliberately 0.x crate metadata); the single source of
    // truth is package.json (same value tauri.conf.json / package_info() use). Best-effort parse:
    // "unknown" if the file is absent (e.g. a stripped source build).
    let app_version = std::fs::read_to_string("../package.json")
        .ok()
        .and_then(|s| {
            let i = s.find("\"version\"")?;
            let after = &s[i + "\"version\"".len()..];
            let start = after.find('"')? + 1;
            let end = after[start..].find('"')? + start;
            Some(after[start..end].to_string())
        })
        .filter(|s| !s.is_empty())
        .unwrap_or_else(|| "unknown".into());
    println!("cargo:rustc-env=KITE_APP_VERSION={app_version}");
    println!("cargo:rerun-if-changed=../package.json");

    tauri_build::build()
}
