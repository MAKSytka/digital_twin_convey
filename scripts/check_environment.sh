#!/usr/bin/env bash
set -Eeuo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

printf '%-28s %s\n' "Ubuntu" "$(. /etc/os-release; echo "${PRETTY_NAME:-unknown}")"
printf '%-28s %s\n' "Python" "$(python3 --version 2>&1)"

if [[ -f /opt/ros/jazzy/setup.bash ]]; then
  set +u
  source /opt/ros/jazzy/setup.bash
  set -u
  printf '%-28s %s\n' "ROS_DISTRO" "${ROS_DISTRO:-unknown}"
  printf '%-28s %s\n' "ros_gz_sim" "$(ros2 pkg prefix ros_gz_sim 2>/dev/null || echo missing)"
  printf '%-28s %s\n' "ros_gz_bridge" "$(ros2 pkg prefix ros_gz_bridge 2>/dev/null || echo missing)"
else
  echo "ROS 2 Jazzy is not installed." >&2
fi

if command -v gz >/dev/null 2>&1; then
  gz sim --version || true
else
  echo "Gazebo 'gz' command is missing." >&2
fi

if python3 -c "import yaml" >/dev/null 2>&1; then
  python3 "$PROJECT_ROOT/tools/validate_project.py"
else
  echo "Static validation skipped: install python3-yaml first." >&2
fi
