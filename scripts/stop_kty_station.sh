#!/usr/bin/env bash
set -Eeuo pipefail

pkill -f "kty_station.launch.py" 2>/dev/null || true
pkill -f "gz sim.*kty_station.sdf" 2>/dev/null || true
pkill -f "station_controller" 2>/dev/null || true
pkill -f "product_spawner" 2>/dev/null || true
pkill -f "depth_perception" 2>/dev/null || true
pkill -f "safety_monitor" 2>/dev/null || true
pkill -f "metrics_node" 2>/dev/null || true
