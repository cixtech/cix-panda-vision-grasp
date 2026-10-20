# Copyright 2026 Cix Technology Group Co., Ltd.
# Licensed under the Apache License, Version 2.0

"""Spawn poses for movable objects in worlds/arm_on_the_table.sdf (shared with commander)."""

# (name, x, y, z, roll, pitch, yaw) in Gazebo world frame; x/y match panda_link0 table plane.
WORLD_OBJECT_POSES = (
    ("bolt1", 0.55, 0.42, 1.05, 0.0, 1.5, 0.0),
    ("bolt2", 0.42, 0.72, 1.05, 0.0, -1.5, -2.82),
    ("hammer1", 0.52, 0.28, 1.078, 1.5708, 0.0, 0.1),
    ("cole_hammer1", 0.38, 0.54, 1.03, 0.0, 0.0, 0.25),
    ("monkey_wrench1", 0.28, 0.32, 1.024, 0.0, 0.0, -0.55),
)
