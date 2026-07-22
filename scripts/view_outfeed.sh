#!/usr/bin/env bash
set -Eeuo pipefail

# Move the Gazebo GUI camera to the downstream conveyor added after the throat.
gz service \
  -s /gui/move_to \
  --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean \
  -r 'data: "outfeed_conveyor"' \
  --timeout 5000
