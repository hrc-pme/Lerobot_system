#!/usr/bin/env bash
set -e

# Source ROS
if [ -n "${ROS_DISTRO:-}" ] && [ -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]; then
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
elif [ -f "/opt/ros/humble/setup.bash" ]; then
  source "/opt/ros/humble/setup.bash"
fi

# Activate venv
if [ -n "${VENV_PATH:-}" ] && [ -f "${VENV_PATH}/bin/activate" ]; then
  source "${VENV_PATH}/bin/activate"
elif [ -f "/opt/venv/bin/activate" ]; then
  source "/opt/venv/bin/activate"
fi

exec "$@"
