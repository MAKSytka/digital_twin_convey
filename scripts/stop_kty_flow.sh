#!/usr/bin/env bash
set -Eeuo pipefail

pkill -f "kty_flow.launch.py" 2>/dev/null || true
pkill -f "kty_flow_cycle" 2>/dev/null || true
pkill -f "kty_flow.sdf" 2>/dev/null || true

echo "KTY flow processes stopped."
