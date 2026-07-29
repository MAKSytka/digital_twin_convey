#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

failures=0

wait_for_node() {
  local node="$1"
  local deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    if ros2 node list 2>/dev/null | grep -Fxq "$node"; then
      echo "OK node: $node"
      return 0
    fi
    sleep 1
  done
  echo "FAIL node: $node" >&2
  failures=$((failures + 1))
  return 1
}

wait_for_topic() {
  local topic="$1"
  local deadline=$((SECONDS + 20))
  while (( SECONDS < deadline )); do
    if ros2 topic list 2>/dev/null | grep -Fxq "$topic"; then
      echo "OK topic: $topic"
      return 0
    fi
    sleep 1
  done
  echo "FAIL topic: $topic" >&2
  failures=$((failures + 1))
  return 1
}

printf 'ROS nodes:\n'
wait_for_node /kty_flow_cycle || true
wait_for_node /kty_depth_perception || true
wait_for_node /kty_contour_recorder || true
wait_for_node /kty_vision_dashboard || true
wait_for_node /kty_vision_rgb_bridge || true
wait_for_node /kty_vision_depth_bridge || true

printf '\nROS topics:\n'
for topic in \
  /kty/vision/image \
  /kty/vision/depth_image \
  /kty/perception/contours \
  /kty/perception/debug_image \
  /kty/vision/dashboard \
  /kty/vision/polygons_json \
  /kty/flow/state; do
  wait_for_topic "$topic" || true
done

printf '\nDashboard frame:\n'
if timeout 12 ros2 topic echo /kty/vision/dashboard --once >/dev/null 2>&1; then
  echo "OK: dashboard image received"
else
  echo "FAIL: no dashboard image received in 12 s" >&2
  failures=$((failures + 1))
fi

printf '\nWaiting for non-empty product polygons:\n'
POLYGON_ECHO="/tmp/kty_vision_polygons_echo.txt"
rm -f "$POLYGON_ECHO"
polygon_seen=false
deadline=$((SECONDS + 90))
while (( SECONDS < deadline )); do
  if timeout 4 ros2 topic echo /kty/vision/polygons_json --once >"$POLYGON_ECHO" 2>/dev/null; then
    if grep -Eq 'product_count[^0-9]*[1-9][0-9]*' "$POLYGON_ECHO"; then
      polygon_seen=true
      break
    fi
  fi
  sleep 2
done

if [[ "$polygon_seen" == true ]]; then
  echo "OK: non-empty polygon frame received"
  sed -n '1,4p' "$POLYGON_ECHO"
else
  echo "FAIL: no non-empty polygon frame received in 90 s" >&2
  failures=$((failures + 1))
fi

printf '\nSaved polygon files:\n'
LATEST_JSON="$HOME/.ros/kty_vision/polygons_latest.json"
HISTORY_JSONL="$HOME/.ros/kty_vision/polygons.jsonl"
if [[ -s "$LATEST_JSON" && -s "$HISTORY_JSONL" ]]; then
  if python3 - "$LATEST_JSON" <<'PY'
import json
import sys
from pathlib import Path

path = Path(sys.argv[1])
document = json.loads(path.read_text(encoding="utf-8"))
products = document.get("products", [])
if not products:
    raise SystemExit("latest JSON has no products")
for product in products:
    polygon = product.get("polygon_m", [])
    if len(polygon) < 3:
        raise SystemExit(f"track {product.get('track_id')} has fewer than 3 polygon points")
print(
    "OK: saved JSON contains "
    f"{len(products)} tracked products and valid polygons"
)
PY
  then
    echo "OK file: $LATEST_JSON"
    echo "OK file: $HISTORY_JSONL"
  else
    echo "FAIL: saved polygon JSON is invalid" >&2
    failures=$((failures + 1))
  fi
else
  echo "FAIL: polygon files are missing or empty" >&2
  failures=$((failures + 1))
fi

printf '\nLatest perception message:\n'
timeout 8 ros2 topic echo /kty/perception/contours --once 2>/dev/null | sed -n '1,80p' || true

printf '\nLatest flow state:\n'
timeout 5 ros2 topic echo /kty/flow/state --once 2>/dev/null || true

printf '\nGazebo camera topics:\n'
gz topic -l 2>/dev/null | grep -E '/kty/vision/(image|depth_image)' || true

if (( failures > 0 )); then
  echo "KTY vision diagnostics failed: ${failures} problem(s)." >&2
  exit 1
fi

echo "KTY vision diagnostics: OK"
