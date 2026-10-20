#!/usr/bin/env python3

from ultralytics import YOLO
import os
import copy
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import sys

_script_dir = os.path.dirname(os.path.abspath(__file__))
if _script_dir not in sys.path:
    sys.path.insert(0, _script_dir)
from image_utils import image_msg_to_bgr
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()


def _resolve_model_path():
    """Resolve best.pt for both source tree and colcon install layouts."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(script_dir, 'best.pt'),
        os.path.normpath(
            os.path.join(script_dir, '..', '..', 'share', 'cix_noe_yolov8obb', 'scripts', 'best.pt')
        ),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(
        'YOLO weights not found. Expected best.pt next to this script or under '
        'share/cix_noe_yolov8obb/scripts/ after colcon build. Checked: ' + ', '.join(candidates)
    )


class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        model_path = _resolve_model_path()
        self.get_logger().info(f'Loading YOLO model: {model_path}')
        self.model = YOLO(model_path)

        self.yolov8_inference = Yolov8Inference()

        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.camera_callback,
            qos_profile_sensor_data,
        )
        self.subscription 

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)

    def camera_callback(self, data):

        try:
            img = image_msg_to_bgr(data, bridge)
        except Exception as exc:
            self.get_logger().warning(
                f'image decode failed ({data.encoding}): {exc}',
                throttle_duration_sec=5.0,
            )
            return
        results = self.model(img, conf = 0.90)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        for r in results:
            if(r.obb is not None):
                boxes = r.obb
                for box in boxes:
                    self.inference_result = InferenceResult()
                    b = box.xyxyxyxy[0].to('cpu').detach().numpy().copy() 
                    c = box.cls
                    conf = float(box.conf.cpu().detach().numpy()) if box.conf is not None else 0.0
                    self.inference_result.class_name = self.model.names[int(c)]
                    self.inference_result.confidence = conf
                    a = b.reshape(1,8)
                    self.inference_result.coordinates = copy.copy(a[0].tolist())
                    self.yolov8_inference.yolov8_inference.append(self.inference_result)
            else:
                camera_subscriber.get_logger().info(f"no_results")

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
        self.img_pub.publish(img_msg)

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
