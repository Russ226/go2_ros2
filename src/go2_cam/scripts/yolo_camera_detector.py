#!/usr/bin/env python3

import threading

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from ultralytics import YOLO


class YoloCameraDetector(Node):
    def __init__(self):
        super().__init__('yolo_camera_detector')

        self.input_topic = self.declare_parameter(
            'input_topic', '/utlidar/camera').value
        self.detection_topic = self.declare_parameter(
            'detection_topic', '/utlidar/camera_dect').value
        self.annotated_topic = self.declare_parameter(
            'annotated_topic', '/utlidar/camera_dect/image').value
        self.model_path = self.declare_parameter(
            'model_path', 'yolo11n.pt').value
        self.confidence = float(self.declare_parameter(
            'confidence', 0.40).value)
        self.device = self.declare_parameter('device', '').value
        self.max_fps = float(self.declare_parameter('max_fps', 10.0).value)

        self.bridge = CvBridge()
        self.model = YOLO(self.model_path)
        self.lock = threading.Lock()
        self.busy = False
        self.last_inference_ns = 0

        self.detection_pub = self.create_publisher(
            Detection2DArray, self.detection_topic, 10)
        self.annotated_pub = self.create_publisher(
            Image, self.annotated_topic, qos_profile_sensor_data)
        self.image_sub = self.create_subscription(
            Image, self.input_topic, self.image_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f'YOLO model={self.model_path}; input={self.input_topic}; '
            f'detections={self.detection_topic}; annotated={self.annotated_topic}')

    def image_callback(self, image_msg: Image):
        now_ns = self.get_clock().now().nanoseconds
        interval_ns = int(1e9 / self.max_fps) if self.max_fps > 0 else 0
        if interval_ns and now_ns - self.last_inference_ns < interval_ns:
            return

        with self.lock:
            if self.busy:
                return
            self.busy = True

        try:
            self.last_inference_ns = now_ns
            bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
            kwargs = {'conf': self.confidence, 'verbose': False}
            if self.device:
                kwargs['device'] = self.device
            result = self.model.predict(bgr, **kwargs)[0]

            detections_msg = Detection2DArray()
            detections_msg.header = image_msg.header
            annotated = bgr.copy()

            if result.boxes is not None:
                for xyxy, confidence, class_id in zip(
                    result.boxes.xyxy.cpu().tolist(),
                    result.boxes.conf.cpu().tolist(),
                    result.boxes.cls.cpu().tolist()):
                    x1, y1, x2, y2 = xyxy
                    class_index = int(class_id)
                    class_name = result.names[class_index]

                    detection = Detection2D()
                    detection.header = image_msg.header
                    detection.bbox.center.position.x = (x1 + x2) / 2.0
                    detection.bbox.center.position.y = (y1 + y2) / 2.0
                    detection.bbox.center.theta = 0.0
                    detection.bbox.size_x = x2 - x1
                    detection.bbox.size_y = y2 - y1

                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = class_name
                    hypothesis.hypothesis.score = float(confidence)
                    detection.results.append(hypothesis)
                    detections_msg.detections.append(detection)

                    label = f'{class_name} {confidence:.2f}'
                    p1 = (round(x1), round(y1))
                    p2 = (round(x2), round(y2))
                    cv2.rectangle(annotated, p1, p2, (0, 255, 0), 2)
                    cv2.putText(
                        annotated, label, (p1[0], max(20, p1[1] - 7)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2,
                        cv2.LINE_AA)

            self.detection_pub.publish(detections_msg)
            annotated_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
            annotated_msg.header = image_msg.header
            self.annotated_pub.publish(annotated_msg)

        except Exception as exc:
            self.get_logger().error(f'YOLO inference failed: {exc}')
        finally:
            with self.lock:
                self.busy = False


def main():
    rclpy.init()
    node = YoloCameraDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
