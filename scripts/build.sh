#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_ROOT"

# ROS 2 setup scripts are not compatible with Bash nounset (`set -u`).
# Temporarily disable it only while sourcing the environment.
set +u
source /opt/ros/jazzy/setup.bash
set -u

rosdep install --from-paths src --ignore-src --rosdistro jazzy -y
colcon build --symlink-install

echo
echo "Build complete."
echo "Run: source ${PROJECT_ROOT}/install/setup.bash"
