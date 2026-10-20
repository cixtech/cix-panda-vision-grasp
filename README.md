# PandaVisionGrasp

Panda arm visual grasping simulation on **ROS 2 Humble + Gazebo Sim 6 + MoveIt2 + YOLO OBB**, adapted for **Ubuntu 22.04 / CIX P1 (aarch64)** with **CIX NPU** inference.

Fork of [laoxue888/Moveit2YoloObb](https://github.com/laoxue888/Moveit2YoloObb) (MIT) with CIX P1 adaptation - see [NOTICE](NOTICE) for the list of changes.

---

## Documentation

| Document | Contents |
|----------|----------|
| [docs/adaptation-and-startup.md](docs/adaptation-and-startup.md) | Build, `start_all.sh` startup, grasp flow, troubleshooting |
| [docs/adaptation-and-startup.cn.md](docs/adaptation-and-startup.cn.md) | 部署与使用说明：moveit_py、本项目、启动验证（中文） |
| [docs/ubuntu22.04-moveit2-source-build.md](docs/ubuntu22.04-moveit2-source-build.md) | Building moveit_py from source on Humble |

---

## Prerequisites

- CIX P1 (CD8180, aarch64) board, Ubuntu 22.04, ROS 2 Humble, Gazebo Sim 6. **Ubuntu 24.04 / ROS 2 Jazzy is not supported** for this standalone demo.
- `moveit_py` built from source: [docs/ubuntu22.04-moveit2-source-build.md](docs/ubuntu22.04-moveit2-source-build.md).
- The [CIX Robotics SDK](https://github.com/cix-robotics/robotics_sdk) package `cix_ros_noe_yolo` (NPU YOLO inference), provided one of two ways:
  1. **SDK deb installed** on the board (`cix-ros-humble-noe-yolo`, installs into `/opt/ros/humble`), or
  2. **SDK source checkout**: clone robotics_sdk, build `ros_packages/` with colcon on the board, and set `ROBOTICS_SDK_ROOT` to the checkout path when building this repo.
  The NPU model `yolo_obb.cix` ships with this repo (`ros2_project/models/yolo_obb.cix`, AGPL-3.0) and `start_all.sh` passes it automatically.

## Quick start

**One command** (from `ros2_project/`):

```bash
# First-time: build moveit_py per docs
cd ~/cix-panda-vision-grasp/ros2_project
chmod +x start_all.sh
./start_all.sh deps              # Align Python deps (NodeGraphQt==0.6.38, etc.)
./start_all.sh build             # colcon build only
./start_all.sh start             # Launch only (NPU YOLO; auto-cleans stale env)
./start_all.sh run               # deps check + build + start
```

> Run `colcon` only under `ros2_project/` (repo root has `COLCON_IGNORE`). Logs: `logs/` at repo root.
>
> **NPU inference**: `cix_ros_noe_yolo` is not vendored - `./start_all.sh build` uses the installed SDK package, or compiles it from an SDK source checkout when `ROBOTICS_SDK_ROOT` is set. `start` / `run` also clear leftover processes and DDS/Iceoryx shared memory before launch.

Press **Ctrl+C** in the start terminal to stop all processes. In the UI, enable **show window**, then click a red OBB to grasp.

Full deployment guide: [docs/adaptation-and-startup.md](docs/adaptation-and-startup.md) ([中文](docs/adaptation-and-startup.cn.md)).

---

## Where to run it

**Run the demo on the board itself**, from a terminal in its desktop session (local GNOME/Wayland login, monitor and keyboard attached). That is the supported setup, and the one to demo: such a terminal already carries the session's display variables, so Qt opens the UI on Wayland and Gazebo uses the Xwayland display with nothing to configure.

**SSH is a fallback, not a supported path.** A remote shell does not inherit those variables, so the GUI has to be reattached by hand. Reading logs over SSH is fine; running or demoing the grasp flow is not. `start_all.sh` does not fill them in.

If you launch over SSH anyway, export them yourself. SSH already provides `XDG_RUNTIME_DIR` through `pam_systemd`; these three are what a remote shell is missing:

| Variable | What it is for | Session value |
|----------|----------------|---------------|
| `WAYLAND_DISPLAY` | Selects the Qt Wayland plugin, so the UI opens a real window | `wayland-0` |
| `DISPLAY` | X11 display for Gazebo, which has no Wayland path | `:0` (Xwayland) |
| `XAUTHORITY` | X11 auth cookie, written per session by Xwayland | newest `/run/user/$(id -u)/.mutter-Xwaylandauth.*` |

```bash
export WAYLAND_DISPLAY=wayland-0
export DISPLAY=:0
export XAUTHORITY=$(ls -t /run/user/$(id -u)/.mutter-Xwaylandauth.* | head -1)
```

Do **not** set `QT_QPA_PLATFORM=xcb` to work around a blank window: the Wayland session is what the UI should use, and forcing `xcb` only adds a `libxcb-cursor0` requirement the board does not otherwise need. See [docs/adaptation-and-startup.md](docs/adaptation-and-startup.md) for troubleshooting.

---

## References

- [Upstream repository](https://github.com/laoxue888/Moveit2YoloObb)

## License

Files inherited from upstream are MIT licensed (see [LICENSE](LICENSE)); CIX modifications are Apache-2.0 (see [NOTICE](NOTICE) for the upstream base commit and the list of changes).

We do not accept third-party contributions at this time.
