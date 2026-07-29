#!/usr/bin/env bash
set -Eeuo pipefail

pkill -f 'kty_smoke.launch.py' 2>/dev/null || true
pkill -f 'kty_station_smoke.sdf' 2>/dev/null || true
pkill -f 'kty_smoke_heartbeat' 2>/dev/null || true

sleep 1

echo "KTY smoke processes stopped"
