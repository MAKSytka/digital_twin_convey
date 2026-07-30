#!/usr/bin/env bash
set -Eeuo pipefail

pkill -f "kty_mechatronics.launch.py" 2>/dev/null || true
pkill -f "kty_mechatronics_cycle" 2>/dev/null || true
pkill -f "kty_fill_estimator" 2>/dev/null || true
pkill -f "kty_depth_perception" 2>/dev/null || true
pkill -f "kty_contour_recorder" 2>/dev/null || true
pkill -f "kty_vision_dashboard" 2>/dev/null || true
pkill -f "ros_gz_image.*image_bridge" 2>/dev/null || true
pkill -f "gz sim.*kty_mechatronics" 2>/dev/null || true

echo "KTY mechatronics processes stopped."
