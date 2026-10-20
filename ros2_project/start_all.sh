#!/usr/bin/env bash
# Copyright 2026 Cix Technology Group Co., Ltd.  All rights reserved.
# Licensed under the Apache License, Version 2.0

# PandaVisionGrasp workspace helper: deps | build | start | run
# colcon must run under ros2_project/ (repo root has COLCON_IGNORE).
set -euo pipefail

ROS_WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${ROS_WS}/.." && pwd)"
GE_DIR="${REPO_ROOT}/GraphExecuter/graph_executer"
MOVEIT_WS="${MOVEIT_WS:-$HOME/ws_moveit}"
LOG_DIR="${REPO_ROOT}/logs"
GAZEBO_WAIT_SEC="${GAZEBO_WAIT_SEC:-30}"

# Pinned versions (must match GraphExecuter/graph_executer/requirements.txt)
REQUIRED_NODEGRAPHQT="0.6.38"
REQUIRED_NUMPY_MAJOR_MAX=1

PROFILE_TIMING="${PROFILE_TIMING:-}"
WORLD="${WORLD:-arm_on_the_table}"
OPEN_RQT_GRAPH="${OPEN_RQT_GRAPH:-0}"
RQT_GRAPH_WAIT_SEC="${RQT_GRAPH_WAIT_SEC:-5}"

usage() {
  cat <<EOF
Usage: ./start_all.sh <command> [options]

Commands:
  deps    Align Python deps (pip install pinned versions + verify)
  build   colcon build this workspace only (no launch)
  start   Start Gazebo + NPU YOLO + GraphExecuter, then interactive control (no build)
  run     deps + build + start

Run from ros2_project/:

  cd ~/cix-panda-vision-grasp/ros2_project
  ./start_all.sh deps                  # first-time / team env sync (recommended)
  ./start_all.sh build
  ./start_all.sh start
  ./start_all.sh run

Options (deps):
  --check-only             Verify versions only; do not pip install

Options (start / run):
  --world=NAME             Gazebo world (default: arm_on_the_table)
  --skip-deps-check        Do not verify Python deps before start
  --rqt-graph              Auto-open rqt_graph (ROS node graph; off by default)
  --no-rqt-graph           Do not auto-open rqt_graph (default; kept for compatibility)
  --profile-timing         Log NPU preprocess/npu/postprocess ms (yolo.log)
  --no-profile-timing      Disable NPU timing logs (overrides default on)

Notes (start / run):
  Before launch, automatically cleans stale Gazebo/YOLO/UI processes,
  iox-roudi, and leftover Fast DDS / Iceoryx files under /dev/shm.
  YOLO inference uses the CIX NPU only (cix_ros_noe_yolo).

Environment:
  MOVEIT_WS       Path to moveit_py workspace (default: ~/ws_moveit)
  ROS_DOMAIN_ID   Default: 61
  RMW_IMPLEMENTATION  Default: rmw_fastrtps_cpp
  GAZEBO_WAIT_SEC Seconds to wait for /commander (default: 30)
  OPEN_RQT_GRAPH  1 to auto-open rqt_graph, 0 to skip (default: 0)
  RQT_GRAPH_WAIT_SEC Seconds after UI start before opening rqt_graph (default: 5)
  PROFILE_TIMING  1 to log NPU stage timings; empty = on; 0 to disable

Pinned Python packages (deps): NodeGraphQt==${REQUIRED_NODEGRAPHQT}, numpy 1.x,
  PySide6, Qt.py, opencv-python. See GraphExecuter/graph_executer/requirements.txt

Logs (start/run): ../logs/gazebo.log, ../logs/yolo.log, ../logs/graph_executer.log

Interactive commands (after start): r=arm reset, o=object reset, a=both, s=status, q=quit

Legacy: ./start_all.sh --no-build  same as  ./start_all.sh start
        ./start_all.sh (no command) same as  ./start_all.sh run
EOF
}

# ROS setup.bash references unset vars; incompatible with set -u.
source_setup() {
  set +u
  # shellcheck source=/dev/null
  source "$1"
  set -u
}

parse_start_options() {
  SKIP_DEPS_CHECK="${SKIP_DEPS_CHECK:-0}"
  while [[ $# -gt 0 ]]; do
    case "$1" in
      --yolo-backend=*|--with-ultralytics)
        echo "ERROR: CPU / Ultralytics YOLO backend is not supported. This demo uses NPU only." >&2
        exit 1
        ;;
      --world=*)
        WORLD="${1#*=}"
        shift
        ;;
      --skip-deps-check)
        SKIP_DEPS_CHECK=1
        shift
        ;;
      --rqt-graph)
        OPEN_RQT_GRAPH=1
        shift
        ;;
      --no-rqt-graph)
        OPEN_RQT_GRAPH=0
        shift
        ;;
      --profile-timing)
        PROFILE_TIMING=1
        shift
        ;;
      --no-profile-timing)
        PROFILE_TIMING=0
        shift
        ;;
      --no-build)
        echo "WARN: --no-build is deprecated; use: ./start_all.sh start" >&2
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "Unknown option: $1" >&2
        usage >&2
        exit 1
        ;;
    esac
  done
}

verify_python_deps() {
  local ok=0
  local ng_ver np_major py_major py_minor

  echo "=== Python dependency check ==="

  py_major="$(python3 -c 'import sys; print(sys.version_info.major)' 2>/dev/null || echo 0)"
  py_minor="$(python3 -c 'import sys; print(sys.version_info.minor)' 2>/dev/null || echo 0)"
  if [[ "$py_major" -eq 3 && "$py_minor" -eq 10 ]]; then
    echo "Python 3.10 OK."
  else
    echo "WARN: Python is ${py_major}.${py_minor}; ROS 2 Humble expects 3.10." >&2
    ok=1
  fi

  ng_ver="$(python3 -c "import importlib.metadata as m; print(m.version('NodeGraphQt'))" 2>/dev/null || true)"
  if [[ -z "$ng_ver" ]]; then
    echo "FAIL: NodeGraphQt not installed." >&2
    echo "      Fix: ./start_all.sh deps" >&2
    ok=1
  elif [[ "$ng_ver" != "$REQUIRED_NODEGRAPHQT" ]]; then
    echo "FAIL: NodeGraphQt is ${ng_ver}; required ${REQUIRED_NODEGRAPHQT}." >&2
    echo "      (0.6.44+ breaks process_graph.json load)" >&2
    echo "      Fix: pip3 install 'NodeGraphQt==${REQUIRED_NODEGRAPHQT}'  or  ./start_all.sh deps" >&2
    ok=1
  else
    echo "NodeGraphQt ${ng_ver} OK."
  fi

  np_major="$(python3 -c "import numpy; print(numpy.__version__.split('.')[0])" 2>/dev/null || echo "?")"
  if [[ "$np_major" == "?" ]]; then
    echo "FAIL: numpy not importable." >&2
    echo "      Fix: ./start_all.sh deps" >&2
    ok=1
  elif [[ "$np_major" -gt "$REQUIRED_NUMPY_MAJOR_MAX" ]]; then
    echo "FAIL: numpy $(python3 -c 'import numpy; print(numpy.__version__)') — need numpy 1.x for ROS cv_bridge." >&2
    echo "      Fix: pip3 install 'numpy>=1.23.0,<2.0'  or  ./start_all.sh deps" >&2
    ok=1
  else
    echo "numpy $(python3 -c 'import numpy; print(numpy.__version__)') OK (<2.0)."
  fi

  if python3 -c "import PySide6" 2>/dev/null; then
    echo "PySide6 $(python3 -c 'import importlib.metadata as m; print(m.version("PySide6"))') OK."
  else
    echo "FAIL: PySide6 not importable." >&2
    echo "      Fix: ./start_all.sh deps" >&2
    ok=1
  fi

  if python3 -c "import PySide2" 2>/dev/null; then
    echo "WARN: PySide2 is installed; may conflict with PySide6 / Qt.py." >&2
    echo "      Fix: pip3 uninstall -y PySide2" >&2
    ok=1
  fi

  if python3 -c "import Qt" 2>/dev/null; then
    echo "Qt.py $(python3 -c 'import importlib.metadata as m; print(m.version("Qt.py"))') OK."
  else
    echo "FAIL: Qt.py not importable." >&2
    echo "      Fix: ./start_all.sh deps" >&2
    ok=1
  fi

  if python3 -c "import cv2" 2>/dev/null; then
    echo "opencv-python $(python3 -c 'import importlib.metadata as m; print(m.version("opencv-python"))') OK."
  else
    echo "FAIL: opencv-python (cv2) not importable." >&2
    echo "      Fix: ./start_all.sh deps" >&2
    ok=1
  fi

  if python3 -c "import importlib.metadata as m; m.version('opencv-python-headless')" 2>/dev/null \
    && python3 -c "import importlib.metadata as m; m.version('opencv-python')" 2>/dev/null; then
    echo "WARN: Both opencv-python and opencv-python-headless installed; may conflict." >&2
    ok=1
  fi

  if [[ "$ok" -eq 0 ]]; then
    echo "Python dependency check passed."
  else
    echo "Python dependency check FAILED. Run: ./start_all.sh deps" >&2
  fi
  return "$ok"
}

cmd_deps() {
  local check_only=0

  while [[ $# -gt 0 ]]; do
    case "$1" in
      --with-ultralytics|--yolo-backend=*)
        echo "ERROR: CPU / Ultralytics YOLO backend is not supported. This demo uses NPU only." >&2
        exit 1
        ;;
      --check-only)
        check_only=1
        shift
        ;;
      -h|--help)
        usage
        exit 0
        ;;
      *)
        echo "Unknown option for deps: $1" >&2
        usage >&2
        exit 1
        ;;
    esac
  done

  if [[ "$check_only" -eq 1 ]]; then
    verify_python_deps
    exit $?
  fi

  if [[ ! -f "${GE_DIR}/requirements.txt" ]]; then
    echo "ERROR: ${GE_DIR}/requirements.txt not found." >&2
    exit 1
  fi

  echo "Installing pinned Python deps from ${GE_DIR}/requirements.txt ..."
  pip3 install -r "${GE_DIR}/requirements.txt"

  verify_python_deps || true

  chmod +x "${GE_DIR}/run.sh" 2>/dev/null || true

  cat <<EOF

deps done.

Do NOT run (breaks this project):
  pip3 install -U NodeGraphQt     # use ==${REQUIRED_NODEGRAPHQT} only
  pip3 install 'numpy>=2'         # breaks ROS cv_bridge on Humble

Next: ./start_all.sh build && ./start_all.sh start
EOF
}

cmd_build() {
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "ERROR: /opt/ros/humble/setup.bash not found. Install ROS 2 Humble first." >&2
    exit 1
  fi

  echo "Building in ${ROS_WS} ..."
  cd "${ROS_WS}"
  source_setup /opt/ros/humble/setup.bash
  if [[ -f "${MOVEIT_WS}/install/setup.bash" ]]; then
    source_setup "${MOVEIT_WS}/install/setup.bash"
  else
    echo "WARN: ${MOVEIT_WS}/install/setup.bash not found; build continues without moveit_py." >&2
  fi
  # cix_ros_noe_yolo is NOT vendored in this repo. Two ways to provide it:
  #   1. installed SDK package (cix-ros-humble-noe-yolo deb, or a sourced underlay)
  #   2. ROBOTICS_SDK_ROOT pointing at a robotics_sdk checkout -> build from source
  local pkgs=(yolov8_msgs cix_noe_yolov8obb panda_moveit_config robot_description)
  local sdk_yolo_dir="${ROBOTICS_SDK_ROOT:-}/ros_packages/cix_ros_noe_yolo"
  if ros2 pkg prefix cix_ros_noe_yolo >/dev/null 2>&1; then
    echo "cix_ros_noe_yolo: using installed package ($(ros2 pkg prefix cix_ros_noe_yolo))"
    colcon build --symlink-install --packages-select "${pkgs[@]}"
  elif [[ -f "${sdk_yolo_dir}/package.xml" ]]; then
    echo "cix_ros_noe_yolo: building from ${sdk_yolo_dir}"
    colcon build --symlink-install \
      --base-paths . "${sdk_yolo_dir}" \
      --packages-select cix_ros_noe_yolo "${pkgs[@]}"
  else
    echo "ERROR: cix_ros_noe_yolo not found. Either:" >&2
    echo "  - install the CIX robotics SDK deb (cix-ros-humble-noe-yolo), or" >&2
    echo "  - set ROBOTICS_SDK_ROOT to a robotics_sdk checkout containing ros_packages/cix_ros_noe_yolo." >&2
    exit 1
  fi
  chmod +x src/panda_moveit_config/scripts/arm_control_from_UI.py
  echo "Build done. Source: source ${ROS_WS}/install/setup.bash"
  echo "Next: ./start_all.sh start   or   ./start_all.sh run"
}

setup_env() {
  if [[ ! -f "${ROS_WS}/install/setup.bash" ]]; then
    echo "ERROR: ${ROS_WS}/install/setup.bash not found. Run: ./start_all.sh build" >&2
    exit 1
  fi
  source_setup /opt/ros/humble/setup.bash
  if [[ -f "${MOVEIT_WS}/install/setup.bash" ]]; then
    source_setup "${MOVEIT_WS}/install/setup.bash"
  else
    echo "WARN: ${MOVEIT_WS}/install/setup.bash not found; MoveIt Python may be unavailable." >&2
  fi
  source_setup "${ROS_WS}/install/setup.bash"
  source_setup "${ROS_WS}/install/panda_moveit_config/share/panda_moveit_config/config/ros2_dds_localhost.sh"
  export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-61}"
  # Prefer Fast DDS: leftover CycloneDDS / Iceoryx state from other demos can
  # make rmw_create_node fail with "failed to create domain".
  export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
  # Board bring-up over SSH: Gazebo GUI / UI windows need the board's own display.
  export DISPLAY="${DISPLAY:-:0}"
  # CIX NPU runtime libs (libnoe.so / libaipudrv.so) for the NPU YOLO backend
  # (cix_ros_noe_yolo). Without this the node dies in EngineInfer with
  # "noe_init_context fail".
  if [[ -d /usr/share/cix/lib ]]; then
    export LD_LIBRARY_PATH="/usr/share/cix/lib:${LD_LIBRARY_PATH:-}"
  fi
  if ! ros2 pkg prefix cix_ros_noe_yolo >/dev/null 2>&1; then
    echo "ERROR: cix_ros_noe_yolo is not in this workspace or environment (required for NPU YOLO)." >&2
    echo "  Install the SDK deb (cix-ros-humble-noe-yolo), or build with ROBOTICS_SDK_ROOT set to a robotics_sdk checkout." >&2
    exit 1
  fi
}

show_interactive_help() {
  cat <<'EOF'
=== Interactive control ===
  r   Reset arm (return to place point + open gripper)
  o   Reset objects (bolts/hammers on table to spawn poses)
  a   Reset all (arm first, then objects)
  s   Show process and ROS status
  h   Show this help
  q   Quit and stop all processes
EOF
}

check_bg_pids() {
  local pid
  for pid in "${pids[@]}"; do
    if ! kill -0 "$pid" 2>/dev/null; then
      echo "Process exited unexpectedly (pid ${pid}); see ${LOG_DIR}" >&2
      return 1
    fi
  done
  return 0
}

ros_trigger_service() {
  local service="$1"
  local label="$2"
  if ! ros2 service list 2>/dev/null | grep -Fxq "${service}"; then
    echo "Service ${service} unavailable (ensure Gazebo and /commander are running)" >&2
    return 1
  fi
  echo "${label}..."
  local out msg
  out="$(ros2 service call "${service}" std_srvs/srv/Trigger "{}" 2>&1)" || true
  if echo "${out}" | grep -qiE 'success[=:][[:space:]]*true'; then
    msg="$(echo "${out}" | grep -oE "message[=:][[:space:]]*('[^']*'|\"[^\"]*\"|[^,}\n]+)" | tail -1 \
      | sed -E "s/^message[=:][[:space:]]*//; s/^['\"]//; s/['\"]$//")"
    if [[ -n "${msg}" ]]; then
      echo "Done: ${msg}"
    else
      echo "Done."
    fi
    return 0
  fi
  echo "${out}"
  echo "Failed." >&2
  return 1
}

show_runtime_status() {
  echo "--- Background PIDs ---"
  printf '  %s\n' "${pids[@]}"
  echo "--- ROS nodes ---"
  ros2 node list 2>/dev/null | grep -E 'commander|move_group|yolov8' || echo "  (none detected)"
  echo "--- Reset services ---"
  ros2 service list 2>/dev/null | grep -E '/reset_arm|/reset_objects' || echo "  (not registered)"
}

interactive_control_loop() {
  show_interactive_help
  echo ""
  local need_prompt=1
  while true; do
    if ! check_bg_pids; then
      return 1
    fi
    if [[ "${need_prompt}" -eq 1 ]]; then
      printf "Command [r/o/a/s/h/q]> "
      need_prompt=0
    fi
    if ! IFS= read -r -t 2 cmd; then
      continue
    fi
    need_prompt=1
    cmd="${cmd#"${cmd%%[![:space:]]*}"}"
    cmd="${cmd%"${cmd##*[![:space:]]}"}"
    case "${cmd}" in
      r|R|arm|reset-arm)
        ros_trigger_service /reset_arm "Arm reset" || true
        ;;
      o|O|obj|objects|reset-objects)
        ros_trigger_service /reset_objects "Object reset" || true
        ;;
      a|A|all|reset-all)
        ros_trigger_service /reset_arm "Arm reset" || true
        ros_trigger_service /reset_objects "Object reset" || true
        ;;
      s|S|status)
        show_runtime_status
        ;;
      h|H|help|\?)
        show_interactive_help
        ;;
      q|Q|quit|exit)
        echo "Exiting..."
        return 0
        ;;
      "")
        ;;
      *)
        echo "Unknown command: ${cmd} (type h for help)"
        ;;
    esac
  done
}

cmd_start() {
  parse_start_options "$@"

  local kill_script="${ROS_WS}/scripts/kill_stale_stack.sh"
  if [[ -f "${kill_script}" ]]; then
    echo "Cleaning stale stack processes..."
    bash "${kill_script}"
    echo ""
  fi

  if [[ "${SKIP_DEPS_CHECK:-0}" -eq 0 ]]; then
    if ! verify_python_deps; then
      echo "WARN: Python deps mismatch; GraphExecuter may fail (process_graph.json / UI)." >&2
      echo "      Fix: ./start_all.sh deps" >&2
    fi
    echo ""
  fi

  mkdir -p "$LOG_DIR"
  local pids=()
  local rqt_graph_pid=""

  cleanup() {
    if [[ ${#pids[@]} -eq 0 && -z "${rqt_graph_pid}" ]]; then
      return
    fi
    echo ""
    echo "Stopping all processes..."
    local pid
    if [[ -n "${rqt_graph_pid}" ]] && kill -0 "${rqt_graph_pid}" 2>/dev/null; then
      kill "${rqt_graph_pid}" 2>/dev/null || true
    fi
    for pid in "${pids[@]}"; do
      if kill -0 "$pid" 2>/dev/null; then
        kill "$pid" 2>/dev/null || true
      fi
    done
    # gazebo server/gui can outlive their ros2 launch parent and get orphaned:
    # TERM them directly (never SIGKILL the gui - it soft-resets the Mali GPU
    # and hangs every later gazebo run until reboot).
    pkill -TERM -f "ign gazebo" 2>/dev/null || true
    sleep 3
    pkill -TERM -f "ign gazebo" 2>/dev/null || true
    sleep 2
    if [[ -n "${rqt_graph_pid}" ]]; then
      kill -9 "${rqt_graph_pid}" 2>/dev/null || true
    fi
    for pid in "${pids[@]}"; do
      kill -9 "$pid" 2>/dev/null || true
    done
    wait 2>/dev/null || true
  }

  # HUP included: closing the terminal window must stop the stack too
  # (otherwise gazebo/GraphExecuter keep running as orphans).
  trap cleanup EXIT INT TERM HUP

  start_bg() {
    local name="$1"
    shift
    "$@" >"${LOG_DIR}/${name}.log" 2>&1 &
    pids+=("$!")
    echo "Started ${name} (pid ${pids[-1]}), log: ${LOG_DIR}/${name}.log"
  }

  launch_rqt_graph() {
    if [[ "${OPEN_RQT_GRAPH}" -eq 0 ]]; then
      echo "Skipping rqt_graph (default off; enable with --rqt-graph or OPEN_RQT_GRAPH=1)."
      return 0
    fi
    if [[ -z "${DISPLAY:-}" ]]; then
      echo "WARN: DISPLAY unset; skipping rqt_graph." >&2
      return 0
    fi
    if ! ros2 pkg prefix rqt_graph &>/dev/null; then
      echo "WARN: rqt_graph not installed; skip auto-open." >&2
      echo "      Install: sudo apt install ros-humble-rqt-graph" >&2
      return 0
    fi

    echo "Waiting ${RQT_GRAPH_WAIT_SEC}s for ROS nodes before rqt_graph..."
    sleep "${RQT_GRAPH_WAIT_SEC}"
    echo "Starting rqt_graph (ROS node graph, optional — closing it does not stop the stack)..."
    ros2 run rqt_graph rqt_graph >"${LOG_DIR}/rqt_graph.log" 2>&1 &
    rqt_graph_pid=$!
    echo "Started rqt_graph (pid ${rqt_graph_pid}), log: ${LOG_DIR}/rqt_graph.log"
  }

  setup_env
  ros2 daemon stop
  ros2 daemon start

  echo "Starting Gazebo + MoveIt (world=${WORLD})..."
  start_bg gazebo ros2 launch panda_moveit_config gazebo.launch.py \
    launch_moveit_py:=true "world:=${WORLD}"

  echo "Waiting for /commander (up to ${GAZEBO_WAIT_SEC}s)..."
  local ready=0
  for _ in $(seq 1 "${GAZEBO_WAIT_SEC}"); do
    if ros2 node list 2>/dev/null | grep -q '/commander'; then
      ready=1
      break
    fi
    sleep 1
  done
  if [[ "$ready" -eq 0 ]]; then
    echo "WARN: /commander not seen yet; starting YOLO and UI anyway." >&2
    echo "      Check: tail -f ${LOG_DIR}/gazebo.log" >&2
  fi

  echo "Starting YOLO (NPU)..."
  local profile_timing_enabled=0
  if [[ -z "${PROFILE_TIMING}" ]]; then
    profile_timing_enabled=1
  elif [[ "${PROFILE_TIMING}" == "1" || "${PROFILE_TIMING}" == "true" || "${PROFILE_TIMING}" == "yes" ]]; then
    profile_timing_enabled=1
  fi
  # Bolt OBB model ships with this repo (not bundled in the SDK package).
  local obb_model="${ROS_WS}/models/yolo_obb.cix"
  if [[ ! -f "${obb_model}" ]]; then
    echo "ERROR: NPU YOLO needs ${obb_model}, which is missing." >&2
    return 1
  fi
  local yolo_launch_args=("use_sim_time:=true" "npu_model_path:=${obb_model}")
  if [[ "${profile_timing_enabled}" -eq 1 ]]; then
    yolo_launch_args+=("profile_timing:=true")
    echo "NPU profile_timing enabled -> grep 'inference ms' ${LOG_DIR}/yolo.log"
  fi
  start_bg yolo ros2 launch cix_ros_noe_yolo npu_yolo_obb_inference.launch.py "${yolo_launch_args[@]}"
  sleep 3

  echo "Starting GraphExecuter UI..."
  start_bg graph_executer bash -c "cd '${GE_DIR}' && exec ./run.sh"

  launch_rqt_graph

  cat <<EOF

=== PandaVisionGrasp is running ===
Logs:
  tail -f ${LOG_DIR}/gazebo.log
  tail -f ${LOG_DIR}/yolo.log
  grep 'inference ms' ${LOG_DIR}/yolo.log   # NPU preprocess/npu/postprocess
  tail -f ${LOG_DIR}/graph_executer.log
  tail -f ${LOG_DIR}/rqt_graph.log   # only if started with --rqt-graph

UI: check show window -> hover OBB until red -> click to grasp
ROS graph: not started by default; optional: ./start_all.sh start --rqt-graph
Stop: type q or Ctrl+C

EOF

  interactive_control_loop
}

cmd_run() {
  cmd_deps --check-only || cmd_deps
  cmd_build
  cmd_start "$@"
}

# --- main ---
if [[ $# -eq 0 ]]; then
  cmd_run
  exit 0
fi

case "$1" in
  deps)
    shift
    cmd_deps "$@"
    ;;
  build)
    shift
    if [[ $# -gt 0 ]]; then
      echo "Unknown option for build: $1" >&2
      usage >&2
      exit 1
    fi
    cmd_build
    ;;
  start)
    shift
    cmd_start "$@"
    ;;
  run)
    shift
    cmd_run "$@"
    ;;
  --no-build)
    shift
    cmd_start "$@"
    ;;
  --world=*|--profile-timing|--no-profile-timing)
    cmd_start "$@"
    ;;
  --yolo-backend=*|--with-ultralytics)
    echo "ERROR: CPU / Ultralytics YOLO backend is not supported. This demo uses NPU only." >&2
    exit 1
    ;;
  -h|--help)
    usage
    ;;
  *)
    echo "Unknown command: $1" >&2
    usage >&2
    exit 1
    ;;
esac
