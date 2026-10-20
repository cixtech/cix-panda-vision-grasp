#!/usr/bin/env bash
# Copyright 2026 Cix Technology Group Co., Ltd.  All rights reserved.
# Licensed under the Apache License, Version 2.0

# Kill leftover Gazebo / YOLO / GraphExecuter processes and clear stale DDS /
# Iceoryx shared memory from crashed or unclean sessions.
# Safe to run as normal user (no sudo). Called by start_all.sh before launch.
set +e

echo "Cleaning stale stack processes and DDS shared memory..."

PATTERNS=(
  'ros2 launch panda_moveit_config gazebo.launch.py'
  'ros2 launch cix_noe_yolov8obb cix_noe_yolov8obb.launch.py'
  'ros2 launch cix_ros_noe_yolo npu_yolo_obb_inference.launch.py'
  'arm_control_from_UI.py'
  'cix_noe_yolov8obb_publisher.py'
  'npu_yolo_obb_inference'
  'cix_ros_noe_yolo'
  'ign gazebo'
  'cix-panda-vision-grasp/GraphExecuter'
  'GraphExecuter/graph_executer'
  'graph_executer/main.py'
  'ros2 run rqt_graph rqt_graph'
  'parameter_bridge'
  'move_group'
  'iox-roudi'
)

# SIGTERM the gazebo server/gui first: SIGKILL'ing the gui soft-resets the
# Mali GPU and hangs every later gazebo run until reboot. TERM is enough for
# a healthy gazebo; the -9 sweep below only catches a wedged server.
pkill -TERM -f 'ign gazebo' 2>/dev/null || true
sleep 3

for pat in "${PATTERNS[@]}"; do
  # skip the blanket -9 on gazebo gui (see above); TERM again then -9 server only
  if [[ "$pat" == 'ign gazebo' ]]; then
    pkill -TERM -f "$pat" 2>/dev/null || true
    sleep 2
    pkill -9 -f 'ign gazebo server' 2>/dev/null || true
    continue
  fi
  pkill -9 -f "$pat" 2>/dev/null || true
done

# Iceoryx RouDi and leftover Fast DDS / Iceoryx shm segments can leave the
# next start unable to create an RMW domain (rmw_create_node / CycloneDDS
# "failed to create domain"). Clear them before relaunch.
pkill -9 -f 'iox-roudi' 2>/dev/null || true
rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* \
      /dev/shm/fast_datashm* /dev/shm/iceoryx* 2>/dev/null || true

if command -v ros2 >/dev/null 2>&1; then
  ros2 daemon stop 2>/dev/null || true
fi

echo "Stale stack processes and DDS shared memory cleaned."
