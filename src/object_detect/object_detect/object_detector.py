# object_detector.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data

class ObjectDetector(Node):
    def __init__(self):
        super().__init__('object_detection_node')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.listener_callback,
            qos_profile_sensor_data)
        self.bridge = CvBridge()
        self.model = YOLO("yolov8m.pt")
        self.get_logger().info("🚀 Starting object detection with webcam...")

    def listener_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"Conversion error: {e}")
            return

        # YOLO detection
        results = self.model(frame)
        annotated_frame = results[0].plot()

        # Show only ONE window with detection
        cv2.imshow("YOLOv8 Detection", annotated_frame)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

