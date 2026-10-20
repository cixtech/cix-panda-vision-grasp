#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Decode sensor_msgs/Image from Gazebo ros_gz_bridge (often 8UC3) to BGR."""

import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


def image_msg_to_bgr(msg: Image, bridge: CvBridge) -> np.ndarray:
    """
    Convert sensor_msgs/Image to a contiguous BGR uint8 array.

    ros_gz_bridge on Ignition/Gazebo 6 commonly publishes encoding '8UC3' instead
    of 'bgr8'/'rgb8', which breaks cv_bridge and Qt viewers.
    """
    enc = (msg.encoding or '').strip().lower()
    h, w = int(msg.height), int(msg.width)
    if h <= 0 or w <= 0:
        raise ValueError(f'invalid image size: {w}x{h}')

    if enc in ('8uc3', 'bgr8'):
        step = int(msg.step) if msg.step else w * 3
        expected = step * h
        if len(msg.data) < expected:
            raise ValueError(
                f'buffer too small for {enc}: {len(msg.data)} < {expected}'
            )
        row = np.frombuffer(msg.data, dtype=np.uint8, count=expected).reshape(
            (h, step)
        )
        raw = row[:, : w * 3].reshape((h, w, 3))
        if enc == '8uc3':
            # Ignition/Gazebo R8G8B8 over ros_gz_bridge is RGB byte order.
            return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        return np.ascontiguousarray(raw)

    if enc == 'rgb8':
        rgb = bridge.imgmsg_to_cv2(msg, 'rgb8')
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    if enc in ('rgba8', 'bgra8'):
        img = bridge.imgmsg_to_cv2(msg, 'passthrough')
        if img.shape[2] == 4:
            code = cv2.COLOR_RGBA2BGR if enc == 'rgba8' else cv2.COLOR_BGRA2BGR
            return cv2.cvtColor(img, code)
        return np.ascontiguousarray(img[:, :, :3])

    if enc in ('mono8', '8uc1'):
        gray = np.frombuffer(msg.data, dtype=np.uint8, count=h * w).reshape(
            (h, w)
        )
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

    try:
        return bridge.imgmsg_to_cv2(msg, 'bgr8')
    except Exception:
        img = bridge.imgmsg_to_cv2(msg, 'passthrough')
        if img.ndim == 2:
            return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        if img.shape[2] >= 3:
            return np.ascontiguousarray(img[:, :, :3])
        raise ValueError(f'unsupported encoding: {msg.encoding}') from None
