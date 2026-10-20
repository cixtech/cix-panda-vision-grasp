#!/bin/bash
# Source after install/setup.bash in any terminal that runs ros2 CLI or nodes.
_PKG_SHARE="$(ros2 pkg prefix panda_moveit_config 2>/dev/null)/share/panda_moveit_config"
if [[ -z "$_PKG_SHARE" || ! -d "$_PKG_SHARE" ]]; then
  _SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
  _PKG_SHARE="$(cd "${_SCRIPT_DIR}/.." && pwd)"
fi
export FASTRTPS_DEFAULT_PROFILES_FILE="${_PKG_SHARE}/config/fastdds_shm_only.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="${FASTRTPS_DEFAULT_PROFILES_FILE}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
export ROS_LOCALHOST_ONLY=1
export GZ_IP=127.0.0.1
export IGN_IP=127.0.0.1
