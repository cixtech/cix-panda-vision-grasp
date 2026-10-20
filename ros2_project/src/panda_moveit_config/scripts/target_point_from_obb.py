# Copyright 2026 Cix Technology Group Co., Ltd.
# Licensed under the Apache License, Version 2.0

"""OBB pixel corners -> /target_point [x, y, angle] (matches GraphExecuter click logic)."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import numpy as np

from world_object_poses import WORLD_OBJECT_POSES

# Pinhole model aligned with moveit2_yolobb_ws.py and panda.urdf.xacro camera mount.
# camera_mount_z: panda_link0 z of camera_joint (meters); must match intrinsic "z" below.
CAMERA_MOUNT_Z = 0.45

DEFAULT_INTRINSICS = {
    "fx": 253.93635749816895,
    "fy": 253.93635749816895,
    "cx": 320.0,
    "cy": 240.0,
    "z": CAMERA_MOUNT_Z,
    "init_x": 0.22,
    "init_y": 0.65,
    # Calibrated at Form display resolution; raw /image_raw (480x360) scales into this frame.
    "ref_image_w": 640.0,
    "ref_image_h": 480.0,
}

# Table workspace bounds in panda_link0 (meters) for spatial_hint labels.
DEFAULT_TABLE_BOUNDS = {
    "x_min": 0.25,
    "x_max": 0.58,
    "y_min": 0.28,
    "y_max": 0.75,
}


@dataclass(frozen=True)
class TargetPoint:
    x: float
    y: float
    angle_rad: float
    pixel_center: Tuple[float, float]


def _scale_corners_to_ref(
    corners_4x2: np.ndarray,
    image_w: float,
    image_h: float,
    ref_w: float,
    ref_h: float,
) -> np.ndarray:
    """Scale OBB corners from source image size to reference intrinsics frame."""
    sx = ref_w / max(image_w, 1.0)
    sy = ref_h / max(image_h, 1.0)
    scaled = corners_4x2.astype(np.float64).copy()
    scaled[:, 0] *= sx
    scaled[:, 1] *= sy
    return scaled


def obb_corners_to_target_point(
    corners_4x2: Sequence[Sequence[float]],
    image_w: float,
    image_h: float,
    intrinsics: Optional[Dict[str, float]] = None,
) -> TargetPoint:
    """
    Convert four OBB vertex pixel coordinates to table-frame grasp pose.

    corners_4x2: 4x2 array in source image pixel space (same as /image_raw).
    """
    params = {**DEFAULT_INTRINSICS, **(intrinsics or {})}
    fx = params["fx"]
    fy = params["fy"]
    cx = params["cx"]
    cy = params["cy"]
    z = params["z"]
    init_x = params["init_x"]
    init_y = params["init_y"]
    ref_w = params.get("ref_image_w", 640.0)
    ref_h = params.get("ref_image_h", 480.0)

    corners = np.asarray(corners_4x2, dtype=np.float64).reshape(4, 2)
    points = _scale_corners_to_ref(corners, image_w, image_h, ref_w, ref_h)
    middle = np.sum(points, axis=0) / 4.0

    x = -z * (middle[1] - cy) / fy + init_x
    y = -z * (middle[0] - cx) / fx + init_y

    dist1 = math.hypot(points[0, 0] - points[1, 0], points[0, 1] - points[1, 1])
    dist2 = math.hypot(points[1, 0] - points[2, 0], points[1, 1] - points[2, 1])

    if dist1 > dist2:
        denominator = points[0, 0] - points[1, 0]
        if denominator == 0:
            edge_angle = math.pi / 2
        else:
            edge_angle = math.atan2(points[0, 1] - points[1, 1], denominator)
    else:
        denominator = points[1, 0] - points[2, 0]
        if denominator == 0:
            edge_angle = math.pi / 2
        else:
            edge_angle = math.atan2(points[1, 1] - points[2, 1], denominator)

    angle_rad = math.pi / 2 - edge_angle
    return TargetPoint(
        x=float(x),
        y=float(y),
        angle_rad=float(angle_rad),
        pixel_center=(float(middle[0]), float(middle[1])),
    )

def compute_spatial_hint(
    x: float,
    y: float,
    table_bounds: Optional[Dict[str, float]] = None,
) -> str:
    """
    Map table (x, y) to a short region label for natural-language matching.

    x: larger -> table front (away from arm base).
    y: larger -> table left (from arm view).
    """
    b = {**DEFAULT_TABLE_BOUNDS, **(table_bounds or {})}
    x_mid = 0.5 * (b["x_min"] + b["x_max"])
    y_mid = 0.5 * (b["y_min"] + b["y_max"])

    x_band = "front" if x >= x_mid else "back"
    y_band = "left" if y >= y_mid else "right"
    return f"{y_band}-{x_band}"


def _object_display_name(model_name: str) -> str:
    """Human-readable label from Gazebo model name (for agent hints only)."""
    base = model_name.rstrip("0123456789")
    if base == "cole_hammer":
        return "cole hammer"
    if base == "monkey_wrench":
        return "monkey wrench"
    return base.replace("_", " ")


def nearest_world_object(
    x: float,
    y: float,
    max_distance: float = 0.12,
) -> Optional[Dict[str, str]]:
    """Match table (x,y) to closest spawn pose; helps when user names object type."""
    best_name = None
    best_dist = float("inf")
    for entry in WORLD_OBJECT_POSES:
        name, wx, wy = entry[0], entry[1], entry[2]
        dist = math.hypot(x - wx, y - wy)
        if dist < best_dist:
            best_dist = dist
            best_name = name
    if best_name is None or best_dist > max_distance:
        return None
    return {
        "world_object_id": best_name,
        "display_name": _object_display_name(best_name),
    }


def yolo_detection_to_candidate(
    detection_id: str,
    corners_4x2: Sequence[Sequence[float]],
    confidence: float,
    image_w: float,
    image_h: float,
    intrinsics: Optional[Dict[str, float]] = None,
) -> Dict:
    """
    Build one grasp candidate from a YOLO OBB (class name ignored — all objects use same label).
    """
    tp = obb_corners_to_target_point(corners_4x2, image_w, image_h, intrinsics)
    hint = compute_spatial_hint(tp.x, tp.y)
    candidate = {
        "id": detection_id,
        "source": "yolo",
        "confidence": round(float(confidence), 4),
        "table_xy": [round(tp.x, 4), round(tp.y, 4)],
        "target_point": [round(tp.x, 4), round(tp.y, 4), round(tp.angle_rad, 4)],
        "pixel_center": [round(tp.pixel_center[0], 1), round(tp.pixel_center[1], 1)],
        "spatial_hint": hint,
    }
    world_hint = nearest_world_object(tp.x, tp.y)
    if world_hint is not None:
        candidate.update(world_hint)
    return candidate
