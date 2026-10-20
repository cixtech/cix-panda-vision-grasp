#!/usr/bin/env bash
# Wrapper: delegates to ros2_project/start_all.sh (colcon runs in that directory).
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/../ros2_project" && pwd)/start_all.sh" "$@"
