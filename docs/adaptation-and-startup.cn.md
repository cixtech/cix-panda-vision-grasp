# PandaVisionGrasp 部署与使用说明（CIX P1 / Ubuntu 22.04）

适用：**Ubuntu 22.04** · **ROS 2 Humble** · **Gazebo Sim 6** · **CIX P1（aarch64）**。本独立仓库 demo **不支持** Ubuntu 24.04 / ROS 2 Jazzy。

**English:** [adaptation-and-startup.md](./adaptation-and-startup.md)

---

## 复现路线图

按顺序执行，每步末尾有**验收命令**。

| 步骤 | 内容 | 频率 |
|------|------|------|
| [§1](#1-系统依赖) | 系统依赖（apt） | 首次 |
| [§2](#2-编译-moveit_py) | 编译 `moveit_py` → `~/ws_moveit` | 首次 |
| [§3](#3-编译本项目) | 编译 PandaVisionGrasp → `ros2_project/` | 首次 / 代码变更后 |
| [§4](#4-启动与验证) | 启动仿真与验证 | 每次使用 |

---

## 1. 系统依赖

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-humble-moveit ros-humble-ros-gz \
  ros-humble-gz-ros2-control ros-humble-pilz-industrial-motion-planner \
  python3-colcon-common-extensions python3-rosdep git

sudo rosdep init   # 若已初始化可跳过
rosdep update
```

---

## 2. 编译 moveit_py

`arm_control_from_UI.py` 依赖 `import moveit`。Humble 的 apt MoveIt **不能**满足此要求；未编译时 launch 会跳过抓取节点：

```
Skipping arm_control_from_UI.py: Python package 'moveit' not found
```

```bash
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone -b humble https://github.com/ros-planning/moveit2.git

cd ~/ws_moveit
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-up-to moveit_py \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 2    # ARM / 内存不足改为 1

source ~/ws_moveit/install/setup.bash
python3 -c "import moveit; print('OK')"
```

**验收：** 最后一行输出 `OK`。

每次新开终端须：

```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
export ROS_DOMAIN_ID=61
```

---

## 3. 编译本项目

克隆仓库后，**仅在** `ros2_project/` 下 `colcon`（根目录有 `COLCON_IGNORE`）。

本项目**不自带** `cix_ros_noe_yolo`（NPU 推理包）：`./start_all.sh build` 默认使用已安装的 SDK 包（`cix-ros-humble-noe-yolo` deb / 已 source 的 underlay）；设置环境变量 `ROBOTICS_SDK_ROOT` 指向 robotics_sdk 源码 checkout 时改为从源码编译。

> `yolo_obb.cix` 模型（单类螺栓 OBB，派生自 Ultralytics 权重，AGPL-3.0）**随本仓库分发**（`ros2_project/models/yolo_obb.cix`），`start_all.sh` 会通过 `npu_model_path` 自动传给 SDK 节点。SDK 的其他 `.cix` 模型（如需使用）用 SDK 包的 `scripts/download_models.sh` 下载。

```bash
cd ~/cix-panda-vision-grasp/ros2_project
chmod +x start_all.sh

./start_all.sh deps                      # 安装 GraphExecuter 锁定依赖

source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
./start_all.sh build

source install/setup.bash
chmod +x src/panda_moveit_config/scripts/arm_control_from_UI.py
```

手动编译（从 robotics_sdk 源码编译 cix_ros_noe_yolo）：

```bash
ROBOTICS_SDK_ROOT=~/robotics_sdk colcon build --symlink-install \
  --base-paths . "${ROBOTICS_SDK_ROOT}/ros_packages/cix_ros_noe_yolo" \
  --packages-select yolov8_msgs cix_ros_noe_yolo cix_noe_yolov8obb panda_moveit_config robot_description
```

| 包 | 版本要求 |
|----|----------|
| NodeGraphQt | **==0.6.38** |
| numpy | **1.x（<2.0）** |

**勿执行：** `pip install -U NodeGraphQt`、`pip install 'numpy>=2'`。

`yolov8_msgs` 符号链接冲突时：

```bash
rm -rf build/yolov8_msgs install/yolov8_msgs
colcon build --symlink-install --packages-select yolov8_msgs
```

**验收：**

```bash
test -f install/setup.bash && echo OK
```

---

## 4. 启动与验证

仿真、YOLO、GraphExecuter 均由 `start_all.sh` 一键启动（含 localhost 网络配置）。请在板子本机的桌面会话里开终端运行，终端自带会话的显示变量；SSH 是备选方案，不是受支持的运行方式。

```bash
cd ~/cix-panda-vision-grasp/ros2_project
./start_all.sh start                       # NPU YOLO；启动前自动清理残留进程与 DDS/shm
```

首次可合并：`./start_all.sh run`（deps → build → start）。

| 选项 | 说明 |
|------|------|
| `--world=arm_on_the_table` | Gazebo 世界名称 |

`start` / `run` 会自动清理残留的 Gazebo/YOLO/UI、`iox-roudi` 以及 `/dev/shm` 下 Fast DDS / Iceoryx 文件，并默认使用 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp`。

等待约 30 s。日志：`~/cix-panda-vision-grasp/logs/`（`gazebo.log`、`yolo.log`、`graph_executer.log`）。停止：启动终端 **Ctrl+C**。

**点选抓取：** GraphExecuter → **show window** → 悬停 OBB 变红 → 单击。

### 验收

**仿真 + 点选：**

```bash
export ROS_DOMAIN_ID=61
ros2 node list | grep -E 'commander|grasp_candidate_bridge'
ros2 topic info /target_point -v          # 订阅方含 commander
```

---

## 5. 数据流

```
Gazebo --/image_raw--> YOLO / GraphExecuter
YOLO --/Yolov8_Inference--> grasp_candidate_bridge --/grasp/candidates--> 外部消费方
GraphExecuter 点选 --/target_point--> commander --> MoveIt --> 机械臂
```

| 话题 | 类型 | 用途 |
|------|------|------|
| `/grasp/candidates` | `std_msgs/String` | `grasp_candidate_bridge` 输出的 JSON 候选（勿订阅 `/Yolov8_Inference`） |
| `/target_point` | `std_msgs/Float64MultiArray` | 触发抓取 `[x, y, angle]` |
| `/grasp/status` | `std_msgs/String` | `running` / `idle` |
| `/image_raw` | `sensor_msgs/Image` | 仿真相机 |

YOLO 对桌面物体使用同一类别标签；匹配请用 `spatial_hint` / `table_xy`，勿依赖类别名。

---

## 6. 常见问题

| 现象 | 处理 |
|------|------|
| `Skipping arm_control_from_UI` | 未完成 [§2](#2-编译-moveit_py)；或终端未 `source ~/ws_moveit/install/setup.bash` |
| `import moveit` 失败 | 确认 `moveit2` 为 `humble` 分支且编译成功 |
| UI 无图像 | 确认使用 `./start_all.sh start` 启动；查 `logs/graph_executer.log` |
| 没有 UI 窗口 / 黑屏，GraphExecuter 日志出现 `xcb-cursor0 or libxcb-cursor0 is needed` | Qt 走到了 `xcb` 插件，该插件需要 `libxcb-cursor0`（镜像里没有），加载失败后退回 `offscreen`。说明终端不在桌面会话里，或者设了 `QT_QPA_PLATFORM=xcb`。请在板子本机的桌面会话里开终端启动；不要设 `QT_QPA_PLATFORM=xcb` |
| SSH 下没有 UI 窗口 / Gazebo 报 `could not connect to display` | 远程 shell 不继承会话的 `WAYLAND_DISPLAY` / `XAUTHORITY`，`start_all.sh` 也不会补。Qt 加载不到平台插件（`qt.qpa.xcb: could not connect to display`），在 `createPlatformIntegration` 里 abort，并把整个 stack 带崩。请在板子本机运行；确实要用 SSH 的话，自己先 export `WAYLAND_DISPLAY` / `DISPLAY` / `XAUTHORITY` |
| 点选不抓 | 无 `/commander` → §2；有节点 → 查 `logs/gazebo.log` 规划失败 |
| CIX 无检测框 | 检查 NPU YOLO 是否在跑、`yolo.log` 与 `yolo_obb.cix` 模型路径 |
| `/grasp/candidates` 为空 | 仿真未启动或 `grasp_candidate_bridge` 离线 |
| 长时间运行 Gazebo 崩溃 | `./start_all.sh stop && ./start_all.sh start` |

---

## 7. 关键文件

| 说明 | 路径 |
|------|------|
| 一键启动 | `ros2_project/start_all.sh` |
| 自动抓取 | `ros2_project/src/panda_moveit_config/scripts/arm_control_from_UI.py` |
| 候选桥接 | `ros2_project/src/panda_moveit_config/scripts/grasp_candidate_bridge.py` |
| 仿真 launch | `ros2_project/src/panda_moveit_config/launch/gazebo.launch.py` |
| Localhost 网络 | `ros2_project/src/panda_moveit_config/config/ros2_dds_localhost.sh` |
| YOLO launch | `ros2_project/src/cix_noe_yolov8obb/launch/cix_noe_yolov8obb.launch.py` |
| UI | `GraphExecuter/graph_executer/run.sh` |

---

## 参考

- 上游：https://github.com/laoxue888/Moveit2YoloObb  
- MoveIt2 humble：https://github.com/ros-planning/moveit2/tree/humble
