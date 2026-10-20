# ros2_project — ROS 2 workspace

Full guide: [docs/adaptation-and-startup.md](../docs/adaptation-and-startup.md) / [中文](../docs/adaptation-and-startup.cn.md).

**Run this on the board itself**, from a terminal in its desktop session. SSH is a fallback, not a supported path; see [Where to run it](../README.md#where-to-run-it).

**One-command launch** (recommended):

```bash
cd ~/cix-panda-vision-grasp/ros2_project
chmod +x start_all.sh
./start_all.sh deps     # Align Python deps (required first time)
./start_all.sh build    # colcon build only
./start_all.sh start    # Launch only
./start_all.sh run      # deps + build + start
```

**Manual build only** (must run in this directory; do not run colcon at repo root):

```bash
cd ~/cix-panda-vision-grasp/ros2_project
source /opt/ros/humble/setup.bash
source ~/ws_moveit/install/setup.bash
colcon build --symlink-install --packages-select yolov8_msgs cix_noe_yolov8obb panda_moveit_config robot_description
source install/setup.bash
source install/panda_moveit_config/share/panda_moveit_config/config/ros2_dds_localhost.sh
export ROS_DOMAIN_ID=61
```
