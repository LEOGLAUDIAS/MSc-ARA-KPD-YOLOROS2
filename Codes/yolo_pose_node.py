import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseArray, Pose
from cv_bridge import CvBridge

import cv2
from ultralytics import YOLO


class YoloPoseNode(Node):

    def __init__(self):
        super().__init__('yolo_pose_node')

        self.bridge = CvBridge()
        self.model = YOLO("yolov8n-pose.pt")

        # Subscribe to camera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publish pose keypoints
        self.pose_pub = self.create_publisher(
            PoseArray,
            '/pose_estimation',
            10
        )

        self.get_logger().info("YOLOv8 Pose Node with ROS publishing started")

    def image_callback(self, msg):
        # Convert ROS image to OpenCV
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLOv8-Pose
        results = self.model(frame)

        # If no keypoints detected, do nothing
        if results[0].keypoints is None:
            return

        # Extract keypoints (x, y)
        keypoints = results[0].keypoints.xy.cpu().numpy()

        # Create PoseArray message
        pose_array = PoseArray()
        pose_array.header.stamp = self.get_clock().now().to_msg()
        pose_array.header.frame_id = "camera_link"

        for person in keypoints:
            for kp in person:
                pose = Pose()
                pose.position.x = float(kp[0])
                pose.position.y = float(kp[1])
                pose.position.z = 0.0
                pose_array.poses.append(pose)

        # Publish keypoints to ROS
        self.pose_pub.publish(pose_array)

        # Optional OpenCV visualisation (keep for debugging)
        annotated = results[0].plot()
        cv2.imshow("YOLOv8 Pose Detection", annotated)
        cv2.waitKey(1)


def main():
    rclpy.init()
    node = YoloPoseNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
