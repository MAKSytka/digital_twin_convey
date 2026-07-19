#!/usr/bin/env bash
set -Eeuo pipefail

# CameraTracking exposes /gui/move_to as StringMsg -> Boolean.
gz service \
  -s /gui/move_to \
  --reqtype gz.msgs.StringMsg \
  --reptype gz.msgs.Boolean \
  -r 'data: "roller_throat"' \
  --timeout 5000
