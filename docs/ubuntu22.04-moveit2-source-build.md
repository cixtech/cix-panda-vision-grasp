# Ubuntu 22.04 — Building MoveIt2 (Humble) from Source

Target: **Ubuntu 22.04 + ROS 2 Humble**, to provide **MoveItPy** (`import moveit`) for `arm_control_from_UI.py`.

Startup and three-terminal flow: [adaptation-and-startup.md](./adaptation-and-startup.md) / [中文](./adaptation-and-startup.cn.md).

---

## Why build from source

`arm_control_from_UI.py` needs MoveIt Python (MoveItPy).

On Humble, apt `ros-humble-moveit` runs Gazebo and `move_group`, but usually **cannot** `import moveit`; pip install is unreliable.

Without a source build, launch skips the grasp node:

```
Skipping arm_control_from_UI.py: Python package 'moveit' not found
```

Simulation and YOLO still work, but clicks will not trigger automatic grasping.

---

## Install dependencies

```bash
sudo apt update
sudo apt install -y \
  ros-humble-desktop ros-humble-moveit ros-humble-ros-gz \
  ros-humble-gz-ros2-control ros-humble-pilz-industrial-motion-planner \
  python3-colcon-common-extensions python3-rosdep git

sudo rosdep init
rosdep update
```

---

## Build moveit_py

```bash
mkdir -p ~/ws_moveit/src
cd ~/ws_moveit/src
git clone -b humble https://github.com/ros-planning/moveit2.git

cd ~/ws_moveit
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y

colcon build --packages-up-to moveit_py \
  --cmake-args -DCMAKE_BUILD_TYPE=Release \
  --parallel-workers 2

source ~/ws_moveit/install/setup.bash
python3 -c "import moveit; print('OK')"
```

On ARM or low memory, use `--parallel-workers 1`.

---

## Before each session

```bash
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
```

Recommended in `~/.bashrc` (order matters).

---

## References

- [MoveIt2 humble branch](https://github.com/ros-planning/moveit2/tree/humble)
- [MoveIt2 getting started](https://moveit.picknik.ai/humble/doc/tutorials/getting_started/getting_started.html)
