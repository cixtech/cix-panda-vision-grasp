#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

source_setup /opt/ros/humble/setup.bash
if [[ -f "${REPO_ROOT}/ros2_project/install/setup.bash" ]]; then
  source_setup "${REPO_ROOT}/ros2_project/install/setup.bash"
fi

if [[ -f "${REPO_ROOT}/ros2_project/install/panda_moveit_config/share/panda_moveit_config/config/ros2_dds_localhost.sh" ]]; then
  source_setup "${REPO_ROOT}/ros2_project/install/panda_moveit_config/share/panda_moveit_config/config/ros2_dds_localhost.sh"
elif [[ -f "${REPO_ROOT}/ros2_project/src/panda_moveit_config/config/ros2_dds_localhost.sh" ]]; then
  source_setup "${REPO_ROOT}/ros2_project/src/panda_moveit_config/config/ros2_dds_localhost.sh"
fi

export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
cd "${SCRIPT_DIR}"
exec python3 main.py "$@"
