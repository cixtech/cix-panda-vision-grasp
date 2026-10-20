# PandaVisionGrasp — Adaptation and Startup Guide

Target: **Ubuntu 22.04 / CIX P1 (aarch64)**, ROS 2 Humble, Gazebo Sim 6. **Ubuntu 24.04 / ROS 2 Jazzy is not supported** for this standalone demo.  
Features: YOLO OBB vision (CIX NPU) + MoveIt2 automatic grasp simulation.

**中文:** [adaptation-and-startup.cn.md](./adaptation-and-startup.cn.md)（部署与使用说明）

MoveIt2 source build: [ubuntu22.04-moveit2-source-build.md](./ubuntu22.04-moveit2-source-build.md).

---

## 1. Project layout

```
cix-panda-vision-grasp/
├── ros2_project/src/          # Gazebo, MoveIt, YOLO, commander
├── GraphExecuter/graph_executer/  # UI: click OBB -> /target_point
├── docs/
└── ~/ws_moveit/             # moveit_py built from source (see build guide)
```

| Component | Role |
|-----------|------|
| `panda_moveit_config` | Simulation launch, world, automatic grasp |
| `cix_noe_yolov8obb` | YOLO OBB detection |
| `GraphExecuter` | Image display; click target publishes `/target_point` |

---

## 2. Environment

| Item | Requirement |
|------|-------------|
| OS | Ubuntu 22.04 |
| ROS | Humble |
| Simulation | Gazebo Sim 6 (`ros-humble-ros-gz`) |
| MoveIt | apt + source `moveit_py` |
| Domain ID | `ROS_DOMAIN_ID=61` on every terminal |

GraphExecuter Python deps (**pinned; team must match**):

```bash
cd ~/cix-panda-vision-grasp/ros2_project
./start_all.sh deps                      # install + verify (recommended)
./start_all.sh deps --check-only         # verify only
```

| Package | Requirement | If wrong |
|---------|-------------|----------|
| NodeGraphQt | **==0.6.38** | `process_graph.json` fails to load; no yolo node |
| numpy | **1.x (<2.0)** | cv_bridge / UI shows no image |
| PySide6 + Qt.py | see `requirements.txt` | Qt conflicts, GraphExecuter crash |
| Do not install PySide2 | — | conflicts with PySide6 |

Avoid: `pip install -U NodeGraphQt`, `pip install 'numpy>=2'`.

---

## 3. Build this project

Run `colcon` only under `ros2_project/` (repo root has `COLCON_IGNORE`).

`cix_ros_noe_yolo` is **not vendored** here. `./start_all.sh build` uses the installed SDK package (`cix-ros-humble-noe-yolo` deb / sourced underlay) by default, or compiles it from an SDK source checkout when `ROBOTICS_SDK_ROOT` points at a robotics_sdk clone. For a manual build:

> The `yolo_obb.cix` model (single-class bolt OBB, Ultralytics-derived, AGPL-3.0) ships with this repo at `ros2_project/models/yolo_obb.cix`; `start_all.sh` passes it to the SDK node via `npu_model_path`. The other SDK `.cix` models (if you use them) are fetched with the SDK package's `scripts/download_models.sh`.

```bash
cd ~/cix-panda-vision-grasp/ros2_project
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash

# SDK source checkout (set ROBOTICS_SDK_ROOT to your robotics_sdk clone):
# ROBOTICS_SDK_ROOT=~/robotics_sdk colcon build --symlink-install \
#   --base-paths . "${ROBOTICS_SDK_ROOT}/ros_packages/cix_ros_noe_yolo" \
#   --packages-select yolov8_msgs cix_ros_noe_yolo cix_noe_yolov8obb panda_moveit_config robot_description

# default (cix_ros_noe_yolo deb already installed): omit it
colcon build --symlink-install \
  --packages-select yolov8_msgs cix_noe_yolov8obb panda_moveit_config robot_description

source install/setup.bash
chmod +x src/panda_moveit_config/scripts/arm_control_from_UI.py
```

If `yolov8_msgs` symlink conflicts:

```bash
rm -rf build/yolov8_msgs install/yolov8_msgs
colcon build --symlink-install --packages-select yolov8_msgs
```

---

## 4. Networking (localhost, no LAN multicast)

The simulation launch sets localhost networking for child processes. **Each manual terminal** must load the same config or ROS nodes will not see each other and GraphExecuter may show no image.

### 4.1 One-shot script

After install:

```text
~/cix-panda-vision-grasp/ros2_project/install/panda_moveit_config/share/panda_moveit_config/config/ros2_dds_localhost.sh
```

The script sets:

| Variable | Purpose |
|----------|---------|
| `FASTRTPS_DEFAULT_PROFILES_FILE` | FastDDS on 127.0.0.1 only |
| `ROS_LOCALHOST_ONLY=1` | ROS 2 localhost-only |
| `GZ_IP` / `IGN_IP=127.0.0.1` | Gazebo transport stays on loopback |

### 4.2 Recommended: every terminal

```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
source ~/cix-panda-vision-grasp/ros2_project/install/setup.bash
source ~/cix-panda-vision-grasp/ros2_project/install/panda_moveit_config/share/panda_moveit_config/config/ros2_dds_localhost.sh
export ROS_DOMAIN_ID=61

# After changing network config, restart daemon
ros2 daemon stop
ros2 daemon start
```

### 4.3 Verify multicast does not leak to physical NIC

While simulation runs, in another terminal:

```bash
sudo tcpdump -ni enp97s0 'udp and ip multicast' -c 10
```

Expected: **no** `enp97s0 Out ... 239.255.x.x`.

Loopback multicast on `lo` (`127.0.0.1 > 239.255.0.7`) is normal.

---

## 5. Launch

### 5.0 Script launch (recommended)

Run this from a terminal in the board's own desktop session, which already carries the session's display variables. SSH is a fallback, not a supported launch path.

From **`ros2_project/`**:

```bash
cd ~/cix-panda-vision-grasp/ros2_project
chmod +x start_all.sh

./start_all.sh deps                     # align Python deps (first time / teammates)
./start_all.sh build                    # colcon only
./start_all.sh start                    # launch only (requires prior build)
./start_all.sh run                      # deps + build + start
```

From repo root: `./scripts/start_all.sh <command>` (forwards to `ros2_project/start_all.sh`).

`start` / `run` load localhost networking and start simulation, NPU YOLO, and GraphExecuter in the background; logs under `../logs/`. Before launch they also clear stale Gazebo/YOLO/UI processes, `iox-roudi`, and leftover Fast DDS / Iceoryx files under `/dev/shm`.

| Command / option | Description |
|----------------|-------------|
| `deps` | Install pinned `requirements.txt` and verify |
| `deps --check-only` | Verify only |
| `build` | `colcon build` only |
| `start` | Launch only; checks Python deps first; NPU YOLO |
| `run` | `deps` if needed → `build` → `start` |
| `--world=arm_on_the_table` | Gazebo world name |

Legacy: no args = `run`; `--no-build` = `start`.

Env vars: `MOVEIT_WS` (default `~/ws_moveit`), `ROS_DOMAIN_ID` (default `61`), `RMW_IMPLEMENTATION` (default `rmw_fastrtps_cpp`), `GAZEBO_WAIT_SEC` (wait for `/commander`, default 30).

**Ctrl+C** in the start terminal stops all children. Logs:

```bash
tail -f logs/gazebo.log
tail -f logs/yolo.log
tail -f logs/graph_executer.log
```

### 5.1 Three-terminal launch (manual debug)

Assume section 4.2 was sourced in each terminal.

### Terminal A: simulation + MoveIt + grasp

```bash
ros2 launch panda_moveit_config gazebo.launch.py launch_moveit_py:=true
```

Optional:

```bash
ros2 launch panda_moveit_config gazebo.launch.py \
  launch_moveit_py:=true \
  world:=arm_on_the_table \
  use_rviz:=false \
  use_cpu_affinity:=false
```

Checks:

- Log must **not** show `Skipping arm_control_from_UI`
- `ros2 node list` includes `/commander`, `/move_group`
- Launch log shows `[network] localhost only (ROS DDS + Gazebo): ...fastdds_shm_only.xml`

### Terminal B: YOLO (NPU)

```bash
# yolo_obb.cix ships under ros2_project/models/
ros2 launch cix_ros_noe_yolo npu_yolo_obb_inference.launch.py \
  use_sim_time:=true \
  npu_model_path:=$HOME/cix-panda-vision-grasp/ros2_project/models/yolo_obb.cix
```

### Terminal C: GraphExecuter (click UI)

```bash
cd ~/cix-panda-vision-grasp/GraphExecuter/graph_executer
./run.sh
```

`run.sh` sources the ROS workspace and `ros2_dds_localhost.sh`. You can also run `python3 main.py` (loads FastDDS config automatically).

**UI steps:**

1. `res/graphs/process_graph.json` loads on startup
2. Only **one** `yolo_obb` node is needed; **no graph wiring**
3. Enable **show window** → image window opens (restored if saved)
4. Hover OBB until red, then **click** → publishes `/target_point`
5. Terminal A `/commander` plans and executes the grasp

### Post-launch checks

```bash
export ROS_DOMAIN_ID=61
ros2 node list | grep -E 'commander|move_group|yolov8|grasp_candidate'
ros2 topic hz /image_raw
ros2 topic hz /Yolov8_Inference
ros2 topic info /target_point -v
```

`/target_point` subscriber should be `/commander`.

---

## 6. Data flow

```
Gazebo --/image_raw--> YOLO / GraphExecuter
YOLO   --/Yolov8_Inference--> GraphExecuter (draw boxes)
YOLO   --/Yolov8_Inference--> grasp_candidate_bridge --/grasp/candidates--> external consumers
GraphExecuter click --/target_point--> commander (arm_control_from_UI.py)
commander --> MoveIt plan/execute --> arm
```

| Topic | Type | Description |
|-------|------|-------------|
| `/image_raw` | sensor_msgs/Image | Sim camera |
| `/Yolov8_Inference` | yolov8_msgs/Yolov8Inference | OBB detections |
| `/grasp/candidates` | std_msgs/String | JSON grasp candidates from `grasp_candidate_bridge` (position-based matching) |
| `/grasp/status` | std_msgs/String | `idle` / `running` (`failed` only if `/reset_arm` cannot reach place) — commander arm state |
| `/inference_result` | sensor_msgs/Image | Annotated image (optional) |
| `/target_point` | std_msgs/Float64MultiArray | x, y, angle (rad, in panda_link0) |

---

## 7. Grasp sequence (7 steps)

Logic: `ros2_project/src/panda_moveit_config/scripts/arm_control_from_UI.py`, launch node name `commander`.

| Step | Action |
|------|--------|
| 1 | Move above target (z=0.18 m) |
| 2 | Open gripper |
| 3 | Descend to pick height (0.119 m) |
| 4 | Close gripper |
| 5 | Lift vertically |
| 6 | Transport horizontally to place point |
| 7 | Open gripper to release |

~3 s after startup the arm moves above the place point; `/target_point` is ignored until homing completes.

---

## 8. Adaptations vs upstream

| Area | Notes |
|------|-------|
| MoveIt Python | Source `moveit_py` on Humble |
| Images | Gazebo `8UC3` → BGR; GraphExecuter dual QoS subscribe |
| Network | localhost DDS + `GZ_IP=127.0.0.1` to avoid LAN multicast |
| NPU | `cix_ros_noe_yolo` + `yolo_obb.cix` |
| Scene objects | bolts, hammers, monkey_wrench, etc. |
| Camera mount | `panda.urdf.xacro` `camera_mount_z` (0.5 m); sync `CAMERA_MOUNT_Z` / `DEFAULT_INTRINSICS["z"]` in `target_point_from_obb.py` and `moveit2_yolobb_ws.py` `self.z` |

---

## 9. Troubleshooting

### MoveIt and grasp

| Symptom | Fix |
|---------|-----|
| `import moveit` fails | `source ~/ws_moveit/install/setup.bash` |
| Skips `arm_control_from_UI` | Same, or `launch_moveit_py:=true` |
| Click does nothing | Ensure `/commander` exists; check terminal A planning log |

### YOLO and UI

| Symptom | Fix |
|---------|-----|
| UI no image | All terminals `source ros2_dds_localhost.sh`; same `ROS_DOMAIN_ID=61`; `ros2 topic hz /image_raw` |
| Image but no boxes | Terminal B YOLO not running |
| YOLO dies with `noe_init_context fail` / `libaipudrv.so` not found | `export LD_LIBRARY_PATH=/usr/share/cix/lib:$LD_LIBRARY_PATH` (`start_all.sh` sets this; needed for manual launches) |
| Click no response | Terminal A commander not running |
| No UI window or a black window; GraphExecuter logs `xcb-cursor0 or libxcb-cursor0 is needed` | Qt picked the `xcb` plugin, which needs `libxcb-cursor0` (not on the image), and fell back to `offscreen`. The terminal is outside the desktop session, or `QT_QPA_PLATFORM=xcb` is set. Launch from a terminal in the board's own desktop session; do not set `QT_QPA_PLATFORM=xcb` |
| No UI window, or Gazebo reports `could not connect to display`, over SSH | A remote shell does not inherit the session's `WAYLAND_DISPLAY` / `XAUTHORITY`, and `start_all.sh` does not fill them in. Qt then has no platform plugin it can load (`qt.qpa.xcb: could not connect to display`) and aborts in `createPlatformIntegration`, which takes the rest of the stack down with it. Run on the board itself; if you must use SSH, export `WAYLAND_DISPLAY` / `DISPLAY` / `XAUTHORITY` yourself first |

### Network

| Symptom | Fix |
|---------|-----|
| Nodes invisible to each other | `source ros2_dds_localhost.sh` then `ros2 daemon stop && ros2 daemon start` |
| Multicast on physical NIC | `echo $GZ_IP` should be `127.0.0.1`; restart simulation |

### Build

| Symptom | Fix |
|---------|-----|
| colcon at repo root fails | Build only under `ros2_project/` |
| `cix_ros_noe_yolo not found` at build/start | Install the `cix-ros-humble-noe-yolo` deb, or set `ROBOTICS_SDK_ROOT` to a robotics_sdk checkout and rebuild |
| `yolo_obb.cix` / model file not found | `ros2_project/models/yolo_obb.cix` ships with this repo; check that it exists |
| `yolov8_msgs` symlink error | `rm -rf build/yolov8_msgs install/yolov8_msgs` and rebuild |

### Long runs and GPU (CIX)

| Symptom | Fix |
|---------|-----|
| Gazebo GUI dies / `VK_ERROR_OUT_OF_DEVICE_MEMORY` | Stop and restart: `./start_all.sh stop && ./start_all.sh start` |
| Sim slows after hours | Same restart; disable display sleep / system suspend while sim runs |
| Need faster sim for testing | Optional performance mode before long sessions (see [§10](./adaptation-and-startup.md#10-long-run-and-performance-cix)) |

---

## 10. Long-run and performance (CIX)

`start_all.sh` does **not** enable performance mode by default. For multi-hour simulation or agent sessions:

1. **Disable screen blanking** and system sleep while Gazebo runs.
2. **Optional — GPU/CPU performance** (requires `sudo`):

```bash
# Mali: keep GPU awake (path may vary by kernel)
echo always_on | sudo tee /sys/class/misc/mali0/device/power_policy

# GPU devfreq (example — list with: ls /sys/class/devfreq/)
echo performance | sudo tee /sys/class/devfreq/CIXH3010/governor
echo performance | sudo tee /sys/class/devfreq/4000/governor
echo performance | sudo tee /sys/class/devfreq/5000/governor

# CPU cpufreq policies (list with: ls /sys/devices/system/cpu/cpufreq/)
for p in /sys/devices/system/cpu/cpufreq/policy*; do
  echo performance | sudo tee "$p/scaling_governor"
done
```

3. **VRAM / ZINK:** faster sim can reach GPU OOM sooner in wall-clock time. Plan periodic `./start_all.sh stop && start` during long sessions.

4. **Revert** to `schedutil` / `coarse_demand` when done if you changed governors for daily desktop use.

---

## 11. Key files

| Purpose | Path |
|---------|------|
| Simulation launch | `ros2_project/src/panda_moveit_config/launch/gazebo.launch.py` |
| Localhost network | `ros2_project/src/panda_moveit_config/config/ros2_dds_localhost.sh` |
| FastDDS profile | `ros2_project/src/panda_moveit_config/config/fastdds_shm_only.xml` |
| Auto grasp | `ros2_project/src/panda_moveit_config/scripts/arm_control_from_UI.py` |
| UI node | `GraphExecuter/graph_executer/nodes/Moveit2YoloObb/moveit2_yolobb_ws.py` |
| UI launcher | `GraphExecuter/graph_executer/run.sh` |
| One-shot start | `ros2_project/start_all.sh` (`scripts/start_all.sh` wrapper) |
| YOLO launch | `ros2_project/src/cix_noe_yolov8obb/launch/cix_noe_yolov8obb.launch.py` |
| World | `ros2_project/src/panda_moveit_config/worlds/arm_on_the_table.sdf` |

---

## 12. References

- [Upstream repository](https://github.com/laoxue888/Moveit2YoloObb)
- [Demo video](https://www.youtube.com/watch?v=ypr3RtJzgKI)
