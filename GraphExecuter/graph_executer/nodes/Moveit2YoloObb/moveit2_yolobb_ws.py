#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import math
import threading
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from NodeGraphQt import BaseNode
from PySide6.QtCore import QObject, QTimer, Qt, QPointF, Signal
from PySide6.QtGui import QBrush, QColor, QImage, QPen, QPixmap, QPolygonF
from PySide6.QtWidgets import QGraphicsPixmapItem, QGraphicsScene, QWidget
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, qos_profile_sensor_data, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray, Float32
from yolov8_msgs.msg import Yolov8Inference

from utils.general import find_nodes_folder

from .ros_image_utils import image_msg_to_bgr

__all__ = ('YoloObbNode',)

# UI refresh cap; FPS label tracks inference rate (NPU: get_cur_fps via /npu_inference_fps).
_DISPLAY_MAX_HZ = 20.0
# /image_raw is 480x360; upscale for Form display (fx/cx/cy use this resolution).
_RAW_W = 480
_RAW_H = 360
_DISPLAY_W = 640
_DISPLAY_H = 480


class FpsCounter:
  def __init__(self, window_s=1.0):
    self._window_s = window_s
    self._times = []

  def tick(self):
    now = time.monotonic()
    self._times.append(now)
    cutoff = now - self._window_s
    self._times = [t for t in self._times if t >= cutoff]
    if len(self._times) < 2:
      return 0.0
    span = self._times[-1] - self._times[0]
    if span <= 0.0:
      return 0.0
    return (len(self._times) - 1) / span


class GraphicsScene(QGraphicsScene):
  def __init__(self, display_widget=None, parent=None):
    QGraphicsScene.__init__(self, parent)
    self.mouse_x = 0
    self.mouse_y = 0
    self._display_widget = display_widget

  def mouseMoveEvent(self, event):
    self.mouse_x = event.scenePos().x()
    self.mouse_y = event.scenePos().y()
    super().mouseMoveEvent(event)

  def mousePressEvent(self, event):
    if (
        self._display_widget is not None
        and self._display_widget.handle_scene_click(event)
    ):
      event.accept()
      return
    super().mousePressEvent(event)


class ImageDisplayWidget(QWidget):
  """Camera + YOLO OBB overlay (single refresh path, no scene.clear flicker)."""

  frame_update_signal = Signal(object, object, float, str)

  def __init__(self, parent=None):
    super(ImageDisplayWidget, self).__init__(parent)
    from .ui.ui_yolo_obb import Ui_YoloObbForm

    self.ui = Ui_YoloObbForm()
    self.ui.setupUi(self)
    self.resize(700, 580)
    self.scene = GraphicsScene(display_widget=self)
    self.ui.graphicsView.setScene(self.scene)
    self.ui.graphicsView.setMouseTracking(True)
    self.frame_update_signal.connect(
      self.update_display, Qt.ConnectionType.QueuedConnection,
    )

    self.bridge = CvBridge()
    self.img = np.zeros([360, 480, 3], dtype=np.uint8)
    self._pixmap_item = None
    self._overlay_items = []
    self._hit_test_rows = []

    self.target_point = [0, 0, 0]
    self.fx = 253.93635749816895
    self.fy = 253.93635749816895
    self.cx = 320
    self.cy = 240
    # Camera-to-table depth; keep in sync with panda.urdf.xacro camera_mount_z / target_point_from_obb.
    self.z = 0.45
    self.init_x = 0.22
    self.init_y = 0.65

  def request_frame_update(self, img, yolo_data, fps, fps_label='FPS'):
    if img is None or img.size == 0:
      return
    self.frame_update_signal.emit(img, yolo_data, fps, fps_label)

  def _clear_overlays(self):
    for item in self._overlay_items:
      self.scene.removeItem(item)
    self._overlay_items.clear()

  def _target_point_from_display_obb(self, points, middle_point):
    """Compute table-frame [x, y, angle] from display-space OBB corners."""
    x = -self.z * (middle_point[1] - self.cy) / self.fy + self.init_x
    y = -self.z * (middle_point[0] - self.cx) / self.fx + self.init_y
    dist1 = math.hypot(points[0][0] - points[1][0], points[0][1] - points[1][1])
    dist2 = math.hypot(points[1][0] - points[2][0], points[1][1] - points[2][1])
    if dist1 > dist2:
      denominator = points[0][0] - points[1][0]
      if denominator == 0:
        edge_angle = math.pi / 2
      else:
        edge_angle = math.atan2(points[0][1] - points[1][1], denominator)
    else:
      denominator = points[1][0] - points[2][0]
      if denominator == 0:
        edge_angle = math.pi / 2
      else:
        edge_angle = math.atan2(points[1][1] - points[2][1], denominator)
    angle = math.pi / 2 - edge_angle
    return [float(x), float(y), float(angle)]

  def _publish_target_point(self, points, middle_point):
    if self.node_obj is None or not getattr(self.node_obj, 'is_created_node', False):
      return False
    self.target_point = self._target_point_from_display_obb(points, middle_point)
    self.node_obj.pub.publish(Float64MultiArray(data=self.target_point))
    return True

  def handle_scene_click(self, event):
    """Left-click inside an OBB publishes /target_point; hover alone never does."""
    if event.button() != Qt.MouseButton.LeftButton:
      return False
    pos = event.scenePos()
    for points, middle_point in self._hit_test_rows:
      qpoly = QPolygonF([QPointF(p[0], p[1]) for p in points])
      if qpoly.containsPoint(pos, Qt.FillRule.OddEvenFill):
        return self._publish_target_point(points, middle_point)
    return False

  def _mark_overlay_non_interactive(self, item):
    item.setAcceptedMouseButtons(Qt.MouseButton.NoButton)

  def update_display(self, img, yolo_data, fps, fps_label='FPS'):
    self.img = np.asarray(img, dtype=np.uint8)
    if self.img.ndim != 3 or self.img.shape[2] != 3:
      return

    display = self.img.copy()
    src_h, src_w = display.shape[:2]
    # Upscale raw (e.g. 480x360) to fixed Form size 640x480 before draw/click math.
    sx = _DISPLAY_W / src_w
    sy = _DISPLAY_H / src_h
    if src_w != _DISPLAY_W or src_h != _DISPLAY_H:
      display = cv2.resize(
        display, (_DISPLAY_W, _DISPLAY_H), interpolation=cv2.INTER_LINEAR,
      )

    if fps > 0.0:
      font_scale = 0.65 if fps_label == 'NPU Inference FPS' else 0.75
      cv2.putText(
        display,
        f'{fps_label}: {fps:.1f}',
        (8, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        font_scale,
        (255, 0, 0),
        2,
        cv2.LINE_AA,
      )

    hover_rows = []
    hit_test_rows = []
    if yolo_data is not None:
      for r in yolo_data.yolov8_inference:
        raw_points = np.array(r.coordinates, dtype=np.float64).reshape([4, 2])
        points = (raw_points * (sx, sy)).astype(np.int32)
        middle_point = np.sum(points, 0) / 4
        hit_test_rows.append((points, middle_point))
        dist = math.sqrt(
          (self.scene.mouse_x - middle_point[0]) ** 2
          + (self.scene.mouse_y - middle_point[1]) ** 2
        )
        conf = float(getattr(r, 'confidence', 0.0))
        label = f'{conf:.2f}'
        box_color = (0, 0, 255) if dist < 15 else (255, 0, 0)
        cv2.putText(
          display,
          label,
          (int(points[0][0]), max(int(points[0][1]) - 6, 14)),
          cv2.FONT_HERSHEY_SIMPLEX,
          0.5,
          box_color,
          1,
          cv2.LINE_AA,
        )
        hover_rows.append((r, points, middle_point, dist))

    self._hit_test_rows = hit_test_rows

    rgb_image = np.ascontiguousarray(cv2.cvtColor(display, cv2.COLOR_BGR2RGB))
    height, width = rgb_image.shape[:2]
    bytes_per_line = 3 * width
    q_image = QImage(
      rgb_image.data, width, height, bytes_per_line, QImage.Format_RGB888,
    ).copy()
    pixmap = QPixmap.fromImage(q_image)

    if self._pixmap_item is None:
      self._pixmap_item = QGraphicsPixmapItem(pixmap)
      self._mark_overlay_non_interactive(self._pixmap_item)
      self.scene.addItem(self._pixmap_item)
    else:
      self._pixmap_item.setPixmap(pixmap)

    self._clear_overlays()

    for r, points, middle_point, dist in hover_rows:
      qpoly = QPolygonF([QPointF(p[0], p[1]) for p in points])

      if dist < 15:
        poly = self.scene.addPolygon(
          qpoly, QPen(QColor(255, 0, 0, 255)), QBrush(QColor(255, 0, 0, 100)),
        )
        self._mark_overlay_non_interactive(poly)
        self._overlay_items.append(poly)
      else:
        poly = self.scene.addPolygon(
          qpoly, QPen(QColor(0, 0, 255, 255)), QBrush(QColor(0, 0, 255, 100)),
        )
        self._mark_overlay_non_interactive(poly)
        self._overlay_items.append(poly)

      dot = self.scene.addEllipse(
        middle_point[0] - 2,
        middle_point[1] - 2,
        4,
        4,
        QPen(Qt.GlobalColor.green),
        QBrush(Qt.GlobalColor.green),
      )
      self._mark_overlay_non_interactive(dot)
      self._overlay_items.append(dot)

  def set_node_obj(self, obj):
    self.node_obj = obj

  def closeEvent(self, event):
    return super().closeEvent(event)


class YoloObbNode(BaseNode, QObject):
  __identifier__ = find_nodes_folder(__file__)[1]
  NODE_NAME = 'yolo_obb'

  def __init__(self):
    super().__init__()
    self.myui = ImageDisplayWidget()
    self.myui.set_node_obj(self)

    self.add_checkbox('open_window', text='show window')
    window_widget = self.get_widget('open_window')
    window_widget.value_changed.connect(self.chk_value_changed)

    self.bridge = CvBridge()
    self.img = np.zeros([360, 480, 3], dtype=np.uint8)
    self._latest_yolo = None
    self._state_lock = threading.Lock()

    self.is_created_node = False
    self._got_frame = False
    self._infer_fps = FpsCounter()
    self._npu_infer_fps = 0.0
    self._npu_fps_last_mono = 0.0
    self._last_display_t = 0.0
    self._display_min_interval = 1.0 / _DISPLAY_MAX_HZ

    self._spin_running = False
    self._spin_thread = None

    self._spin_timer = QTimer()
    self._spin_timer.timeout.connect(self._spin_ros_once)

  def _start_spin_thread(self):
    if self._spin_thread is not None and self._spin_thread.is_alive():
      return
    self._spin_running = True
    self._spin_thread = threading.Thread(
      target=self._spin_loop, name='yolo_obb_ros_spin', daemon=True,
    )
    self._spin_thread.start()

  def _stop_spin_thread(self):
    self._spin_running = False
    thread = self._spin_thread
    if thread is not None and thread.is_alive():
      thread.join(timeout=2.0)
    self._spin_thread = None

  def _spin_loop(self):
    while self._spin_running and rclpy.ok() and self.is_created_node:
      rclpy.spin_once(self.ros_node, timeout_sec=0.01)

  def chk_value_changed(self):
    if self.get_property('open_window'):
      if not self.is_created_node:
        self.create_ros2_node()
      self.myui.show()
      self._spin_timer.stop()
      self._start_spin_thread()
    else:
      self._stop_spin_thread()
      self._spin_timer.stop()
      self.myui.close()

    print('open_window:', self.get_property('open_window'))

  def sync_open_window_from_property(self):
    if self.get_property('open_window'):
      self.chk_value_changed()

  def _spin_ros_once(self):
    if not self.is_created_node:
      return
    rclpy.spin_once(self.ros_node, timeout_sec=0)

  def create_ros2_node(self):
    pre = self.__identifier__.replace('.', '_')
    node_name = f'{pre}_{self.NODE_NAME}_{self.name().replace(" ", "_")}'
    self.ros_node = Node(
      node_name,
      parameter_overrides=[
        Parameter('use_sim_time', Parameter.Type.BOOL, True),
      ],
    )

    image_qos = QoSProfile(
      depth=10,
      reliability=ReliabilityPolicy.RELIABLE,
    )
    self._image_sub = self.ros_node.create_subscription(
      Image,
      '/image_raw',
      self.camera_callback,
      image_qos,
    )
    self._yolo_sub = self.ros_node.create_subscription(
      Yolov8Inference, '/Yolov8_Inference', self.yolo_callback, 10,
    )
    self._npu_fps_sub = self.ros_node.create_subscription(
      Float32, '/npu_inference_fps', self._npu_fps_callback, 10,
    )
    self.pub = self.ros_node.create_publisher(
      Float64MultiArray, '/target_point', 10,
    )
    self.is_created_node = True

  def delete_ros2_node(self):
    self._spin_timer.stop()
    self._stop_spin_thread()
    try:
      self.ros_node.destroy_node()
      self.is_created_node = False
    except Exception:
      pass

  def execute(self):
    if not self.is_created_node:
      self.create_ros2_node()
    if not self._spin_running:
      self._spin_ros_once()

  def _refresh_display(self, infer_fps, force=False, fps_label='FPS'):
    with self._state_lock:
      if self.img is None or self.img.size == 0 or int(self.img.max()) == 0:
        return
      img_copy = self.img.copy()
      yolo_data = self._latest_yolo

    now = time.monotonic()
    if not force and (now - self._last_display_t) < self._display_min_interval:
      return
    self._last_display_t = now

    self.myui.request_frame_update(img_copy, yolo_data, infer_fps, fps_label)

  def _npu_fps_callback(self, msg):
    self._npu_infer_fps = float(msg.data)
    self._npu_fps_last_mono = time.monotonic()

  def camera_callback(self, data):
    try:
      img = image_msg_to_bgr(data, self.bridge)
    except Exception as exc:
      if self.is_created_node:
        self.ros_node.get_logger().warning(
          f'/image_raw decode failed ({data.encoding}): {exc}',
          throttle_duration_sec=5.0,
        )
      return

    with self._state_lock:
      self.img = img
      has_yolo = self._latest_yolo is not None

    if not self._got_frame:
      self._got_frame = True
      self.ros_node.get_logger().info(
        f'camera display active ({img.shape[1]}x{img.shape[0]})'
      )

    # Only refresh when YOLO has not published yet; otherwise wait for yolo_callback.
    if not has_yolo:
      self._refresh_display(0.0, force=True)

  def yolo_callback(self, data):
    with self._state_lock:
      self._latest_yolo = data
    infer_fps = self._infer_fps.tick()
    npu_fresh = (
      self._npu_infer_fps > 0.0
      and (time.monotonic() - self._npu_fps_last_mono) < 1.0
    )
    if npu_fresh:
      self._refresh_display(
        self._npu_infer_fps, force=True, fps_label='NPU Inference FPS',
      )
    else:
      self._refresh_display(infer_fps, force=True, fps_label='FPS')

  def set_messageSignal(self, messageSignal):
    self.messageSignal = messageSignal

  def set_widget_parent(self, parent):
    self.setParent(parent)

  def close_node(self):
    self._spin_timer.stop()
    self._stop_spin_thread()
    self.delete_ros2_node()
    self.myui.close()
    del self.myui

  def _del_node(self):
    self._spin_timer.stop()
    self._stop_spin_thread()
    self.delete_ros2_node()
    self.myui.close()
    del self.myui
