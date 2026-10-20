#!/usr/bin/env python3
# Copyright 2026 Cix Technology Group Co., Ltd.
# Licensed under the Apache License, Version 2.0

"""
Bridge YOLO OBB detections to machine-readable grasp candidates.

YOLO labels every table object as the same class; matching is by position only.
Publishes /grasp/candidates (std_msgs/String JSON) for external consumers.
"""

from __future__ import annotations

import json
import os
import sys
from typing import List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String
from yolov8_msgs.msg import Yolov8Inference

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)

from target_point_from_obb import (  # noqa: E402
    DEFAULT_INTRINSICS,
    DEFAULT_TABLE_BOUNDS,
    yolo_detection_to_candidate,
)


class GraspCandidateBridge(Node):

    def __init__(self):
        super().__init__('grasp_candidate_bridge')
        self.declare_parameter('candidates_topic', '/grasp/candidates')
        self.declare_parameter('yolov8_inference_topic', '/Yolov8_Inference')
        self.declare_parameter('image_topic', '/image_raw')
        self.declare_parameter('publish_rate_hz', 2.0)

        self._image_w = float(DEFAULT_INTRINSICS['ref_image_w'])
        self._image_h = float(DEFAULT_INTRINSICS['ref_image_h'])
        self._latest_yolo: Optional[Yolov8Inference] = None

        candidates_topic = (
            self.get_parameter('candidates_topic').get_parameter_value().string_value
        )
        yolo_topic = (
            self.get_parameter('yolov8_inference_topic').get_parameter_value().string_value
        )
        image_topic = (
            self.get_parameter('image_topic').get_parameter_value().string_value
        )

        self._pub = self.create_publisher(String, candidates_topic, 10)
        self.create_subscription(
            Yolov8Inference,
            yolo_topic,
            self._yolo_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            image_topic,
            self._image_cb,
            qos_profile_sensor_data,
        )

        rate_hz = self.get_parameter('publish_rate_hz').value
        rate_hz = max(0.5, min(float(rate_hz), 20.0))
        self.create_timer(1.0 / rate_hz, self._publish_candidates)

        self.get_logger().info(
            f'Publishing grasp candidates on {candidates_topic} '
            f'(YOLO: {yolo_topic}, image: {image_topic}, {rate_hz:.1f} Hz)'
        )

    def _image_cb(self, msg: Image):
        if msg.width > 0 and msg.height > 0:
            self._image_w = float(msg.width)
            self._image_h = float(msg.height)

    def _yolo_cb(self, msg: Yolov8Inference):
        self._latest_yolo = msg
        self._publish_candidates()

    def _build_payload(self) -> dict:
        candidates: List[dict] = []
        if self._latest_yolo is not None:
            for idx, det in enumerate(self._latest_yolo.yolov8_inference):
                coords = np.asarray(det.coordinates, dtype=np.float64).reshape(-1)
                if coords.size < 8:
                    continue
                corners = coords[:8].reshape(4, 2)
                candidates.append(
                    yolo_detection_to_candidate(
                        detection_id=f'yolo_{idx}',
                        corners_4x2=corners,
                        confidence=float(det.confidence),
                        image_w=self._image_w,
                        image_h=self._image_h,
                    )
                )

        return {
            'frame': 'panda_link0',
            'note': (
                'YOLO uses one class label for all table objects; '
                'match by spatial_hint, table_xy, display_name (optional), or camera image.'
            ),
            'coordinate_guide': {
                'x': 'larger = toward table front (away from arm base)',
                'y': 'larger = toward table left (from arm view)',
            },
            'table_bounds': DEFAULT_TABLE_BOUNDS,
            'image_size': [int(self._image_w), int(self._image_h)],
            'detection_count': len(candidates),
            'candidates': candidates,
        }

    def _publish_candidates(self):
        payload = self._build_payload()
        msg = String()
        msg.data = json.dumps(payload, separators=(',', ':'))
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GraspCandidateBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
