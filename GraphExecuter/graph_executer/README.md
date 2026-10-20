# GraphExecuter (PandaVisionGrasp UI)

Displays camera/YOLO output; clicking an OBB publishes `/target_point`.

## Dependencies

Versions are pinned in `requirements.txt`. Use the repo helper script:

```bash
cd ~/cix-panda-vision-grasp/ros2_project
./start_all.sh deps
./start_all.sh deps --check-only   # verify only
```

Manual install:

```bash
pip install -r requirements.txt
```

**NodeGraphQt must be 0.6.38** (do not `pip install -U NodeGraphQt`).

Inference runs in **ROS 2**, not inside GraphExecuter:

```bash
ros2 launch cix_noe_yolov8obb cix_noe_yolov8obb.launch.py
```

NPU (`.cix`) OBB inference uses `cix_ros_noe_yolo`:

```bash
ros2 launch cix_ros_noe_yolo npu_yolo_obb_inference.launch.py
```

## Run

See [docs/adaptation-and-startup.md](../../docs/adaptation-and-startup.md) for the full three-terminal flow.

```bash
cd graph_executer
./run.sh
```

`run.sh` sources the ROS workspace and `ros2_dds_localhost.sh` (localhost DDS + `GZ_IP`).  
Enable **show window**; a single **yolo_obb** node in the graph needs **no wiring**.
