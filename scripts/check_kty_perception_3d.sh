#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

set +u
source /opt/ros/jazzy/setup.bash
source install/setup.bash
set -u

failures=0

wait_for_node() {
  local node="$1"
  local deadline=$((SECONDS + 25))
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

for node in \
  /kty_classical_3d_perception \
  /kty_contour_recorder_3d \
  /kty_vision_dashboard_3d; do
  wait_for_node "$node" || true
done

printf '\nWaiting for extended 3-D planning frame:\n'
if ! timeout 150 python3 - <<'PY'
import rclpy
from rclpy.node import Node
from singulator_interfaces.msg import KtyProductContourArray


class Check(Node):
    def __init__(self):
        super().__init__('kty_3d_acceptance')
        self.ok = False
        self.occluded_contract_ok = True
        self.create_subscription(
            KtyProductContourArray,
            '/kty/perception/contours',
            self.cb,
            10,
        )

    def cb(self, msg):
        visible = [
            item for item in msg.products
            if item.tracking_state != item.STATE_OCCLUDED
        ]
        occluded = [
            item for item in msg.products
            if item.tracking_state == item.STATE_OCCLUDED
        ]
        valid = [
            item for item in visible
            if len(item.polygon.points) >= 3
            and len(item.oriented_rectangle.points) == 4
            and item.estimated_size.x > 0.0
            and item.estimated_size.y > 0.0
            and item.estimated_size.z > 0.0
            and item.surface_normal.z > 0.0
        ]
        grasps = sum(len(item.grasp_candidates) for item in visible)
        occluded_grasps = sum(len(item.grasp_candidates) for item in occluded)
        self.occluded_contract_ok = self.occluded_contract_ok and occluded_grasps == 0
        print(
            f'frame={msg.frame_sequence} visible={len(visible)} '
            f'occluded={len(occluded)} valid_obb={len(valid)} '
            f'grasps={grasps} occluded_grasps={occluded_grasps}',
            flush=True,
        )
        if (
            visible
            and len(valid) == len(visible)
            and grasps > 0
            and self.occluded_contract_ok
        ):
            self.ok = True


rclpy.init()
node = Check()
try:
    while rclpy.ok() and not node.ok:
        rclpy.spin_once(node, timeout_sec=0.5)
finally:
    ok = node.ok
    node.destroy_node()
    rclpy.shutdown()
raise SystemExit(0 if ok else 1)
PY
then
  echo "FAIL: no complete 3-D planning frame" >&2
  failures=$((failures + 1))
else
  echo "OK: 3-D polygons, OBBs, normals and grasp candidates received"
fi

printf '\nLatest JSON schema:\n'
json_sample="$(timeout 8 ros2 topic echo /kty/vision/polygons_json --once 2>/dev/null || true)"
if printf '%s\n' "$json_sample" | grep -q 'kty_carton_instances_3d/v2'; then
  echo "OK: extended JSON schema"
else
  echo "FAIL: extended JSON schema not observed" >&2
  failures=$((failures + 1))
fi

printf '\nSaved planning file:\n'
LATEST_JSON="$HOME/.ros/kty_vision/polygons_latest.json"
if [[ -s "$LATEST_JSON" ]] && python3 - "$LATEST_JSON" <<'PY'
import json
import sys
from pathlib import Path

payload = json.loads(Path(sys.argv[1]).read_text(encoding='utf-8'))
if payload.get('schema') != 'kty_carton_instances_3d/v2':
    raise SystemExit('unexpected schema')
for product in payload.get('products', []):
    state = product.get('tracking_state', 'VISIBLE')
    if len(product.get('oriented_rectangle_m', [])) != 4:
        raise SystemExit(f"track {product.get('track_id')} has invalid OBB")
    if state == 'OCCLUDED' and product.get('grasp_candidates'):
        raise SystemExit(f"occluded track {product.get('track_id')} has grasps")
print(
    'OK: saved file contains '
    f"{payload.get('visible_count', 0)} visible, "
    f"{payload.get('occluded_count', 0)} occluded and "
    f"{payload.get('grasp_candidate_count', 0)} grasp candidates"
)
PY
then
  echo "OK file: $LATEST_JSON"
else
  echo "FAIL: extended planning JSON is missing or invalid" >&2
  failures=$((failures + 1))
fi

if (( failures > 0 )); then
  echo "KTY classical 3-D diagnostics failed: $failures" >&2
  exit 1
fi

echo "KTY classical 3-D perception diagnostics: OK"
