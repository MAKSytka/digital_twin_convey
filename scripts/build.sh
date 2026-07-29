#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

# ROS 2 setup scripts are not compatible with Bash nounset (`set -u`).
set +u
source /opt/ros/jazzy/setup.bash
set -u

# Some minimal Jazzy rosdep databases do not expose an `ament_python` key even
# though the build backend is already installed by ROS.  Keep the dependency in
# package.xml (correct metadata), verify the backend explicitly, and skip only
# that rosdep lookup.
if ! ros2 pkg prefix ament_python >/dev/null 2>&1; then
  echo "ERROR: ROS package ament_python is not installed." >&2
  echo "Install it with: sudo apt install ros-jazzy-ament-python" >&2
  exit 2
fi

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro jazzy \
  --skip-keys ament_python \
  -y

colcon build --symlink-install

echo
echo "Build complete."
echo "Run: source ${PROJECT_ROOT}/install/setup.bash"
