#!/bin/bash
# SPDX-License-Identifier: GPL-3.0-or-later
# Copyright (C) 2026 Marc Hoffmann (b14ckyy)
# ============================================================
# Kite Ground Control — macOS Build Script
# Builds a UNIVERSAL (arm64 + x86_64) .app and .dmg that run on both
# Apple Silicon and Intel Macs.
#
# Recommended: Use "just build-macos" instead.
#
# This produces an UNSIGNED bundle. To sign + notarize for distribution
# (no Gatekeeper warning on other Macs), run "just notarize-macos"
# afterwards — that step needs an Apple Developer account and reads your
# credentials from the environment / a keychain profile (never committed).
# ============================================================
# Prerequisites:
#   - Node.js (LTS)
#   - Rust (via rustup) + both mac targets (this script adds them)
#   - Xcode Command Line Tools (xcode-select --install)
# ============================================================

set -e

echo ""
echo "============================================"
echo " Kite Ground Control — macOS Release Build"
echo "============================================"
echo ""

if command -v just &> /dev/null; then
    echo "[INFO] just is installed - recommended command: just build-macos"
    echo ""
fi

if ! command -v node &> /dev/null; then
    echo "[ERROR] Node.js not found. Install from https://nodejs.org/"
    exit 1
fi

if ! command -v cargo &> /dev/null; then
    echo "[ERROR] Rust/Cargo not found. Install from https://rustup.rs/"
    exit 1
fi

echo "[1/4] Ensuring both macOS Rust targets are installed..."
rustup target add aarch64-apple-darwin x86_64-apple-darwin

echo "[2/4] Installing npm dependencies + fetching bundled ffmpeg..."
npm install
bash "$(dirname "$0")/fetch-ffmpeg-macos.sh"

echo "[3/4] Building universal application with Tauri..."
npm run tauri build -- --target universal-apple-darwin --bundles app dmg

echo "[4/4] Collecting outputs into release/ (unified naming) ..."
bash "$(dirname "$0")/collect-release.sh"
