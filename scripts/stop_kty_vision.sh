#!/usr/bin/env bash
set -Eeuo pipefail

pkill -f "kty_vision.launch.py" 2>/dev/null || true
pkill -f "kty_flow.launch.py" 2>/dev/null || true
pkill -f "kty_station_sim.*flow_cycle" 2>/dev/null || true
pkill -f "kty_station_sim.*depth_perception" 2>/dev/null || true
pkill -f "kty_station_sim.*contour_recorder" 2>/dev/null || true
pkill -f "kty_station_sim.*vision_dashboard" 2>/dev/null || true
pkill -f "ros_gz_image.*image_bridge" 2>/dev/null || true
pkill -f "gz sim.*kty_flow" 2>/dev/null || true
