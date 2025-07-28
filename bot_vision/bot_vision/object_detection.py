#!/home/aditya-pachauri/learning_ros2_ws/.venv/bin/python3
import rclpy
from rclpy.node import Node
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from bot_interfaces.msg import DetectedObject #type: ignore
from builtin_interfaces.msg import Time
from ultralytics import YOLO

class ObjectDetection(Node):
    def __init__(self):
        super().__init__("object_detection")
        self.subscription = self.create_subscription(Image,"/camera/image_raw",self.image_callback,10)
        
        self.detection_publisher = self.create_publisher(DetectedObject,"/uav/detections",10)
        
        self.bridge = CvBridge()
        self.model = YOLO('yolov8s.pt')
        self.get_logger().info("Detection node started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        
        results = self.model.predict(frame, classes=[0, 2])
        images = results[0]

        current_time = self.get_clock().now().to_msg()

        cv2.imshow("detections", images.plot())
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()