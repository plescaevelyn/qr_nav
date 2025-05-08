import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped, TransformStamped
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import csv
from datetime import datetime
import tf2_ros
import tf_transformations

class TagDetectorNode(Node):
    def __init__(self):
        super().__init__('tag_detector')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(PoseStamped, 'tag_detections', 10)
        self.subscription = self.create_subscription(Image, '/oak/rgb/image_raw', self.image_callback, 10)
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.marker_pub = self.create_publisher(Marker, 'tag_markers', 10)

        self.camera_matrix = np.array([[878.0, 0.0, 640.0],
                                       [0.0, 878.0, 360.0],
                                       [0.0, 0.0, 1.0]], dtype=np.float32)
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)
        self.tag_size = 0.05  # meters

        self.aruco_dicts = {
            'apriltag': cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),
            'aruco': cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        }
        self.qr_detector = cv2.QRCodeDetector()

        log_dir = os.path.join(os.path.dirname(__file__), '../logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_path = os.path.join(log_dir, f"tag_detections_{timestamp}.csv")
        with open(self.log_path, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow(['timestamp', 'tag_id', 'x', 'y', 'z'])

        self.get_logger().info("TagDetectorNode started. Logging to CSV, publishing tf and RViz markers.")

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # ArUco / AprilTag
        for tag_type, dictionary in self.aruco_dicts.items():
            corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary)
            if ids is not None:
                for i, corner in enumerate(corners):
                    success, rvec, tvec = cv2.solvePnP(
                        self._get_object_points(), corner[0], self.camera_matrix, self.dist_coeffs
                    )
                    if success:
                        tag_id = f"{tag_type}_{ids[i][0]}"
                        self.publish_pose(tvec, rvec, frame_id='oak_rgb_camera_frame', tag_id=tag_id)

        # QR code
        retval, decoded_info, points, _ = self.qr_detector.detectAndDecodeMulti(gray)
        if retval and points is not None:
            for i, corner in enumerate(points):
                if decoded_info[i]:  # doar coduri valide
                    success, rvec, tvec = cv2.solvePnP(
                        self._get_object_points(), corner.reshape(4, 2), self.camera_matrix, self.dist_coeffs
                    )
                    if success:
                        tag_id = f"qr_{decoded_info[i]}"
                        self.publish_pose(tvec, rvec, frame_id='oak_rgb_camera_frame', tag_id=tag_id)

    def _get_object_points(self):
        half = self.tag_size / 2.0
        return np.array([
            [-half,  half, 0],
            [ half,  half, 0],
            [ half, -half, 0],
            [-half, -half, 0]
        ], dtype=np.float32)

    def publish_pose(self, tvec, rvec, frame_id, tag_id='tag'):
        x, y, z = tvec.flatten()

        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = frame_id
        pose_msg.pose.position.x = x
        pose_msg.pose.position.y = y
        pose_msg.pose.position.z = z

        quat = tf_transformations.quaternion_from_euler(
            float(rvec[0]), float(rvec[1]), float(rvec[2])
        )
        pose_msg.pose.orientation.x = quat[0]
        pose_msg.pose.orientation.y = quat[1]
        pose_msg.pose.orientation.z = quat[2]
        pose_msg.pose.orientation.w = quat[3]

        self.publisher.publish(pose_msg)
        self.get_logger().info(f'Detected {tag_id} at x={x:.2f}, y={y:.2f}, z={z:.2f}')

        with open(self.log_path, 'a', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                datetime.now().isoformat(timespec='seconds'),
                tag_id,
                x, y, z
            ])

        # TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = frame_id
        t.child_frame_id = tag_id
        t.transform.translation.x = x
        t.transform.translation.y = y
        t.transform.translation.z = z
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        self.tf_broadcaster.sendTransform(t)

        # Marker RViz
        self.publish_marker(x, y, z, frame_id, tag_id)

    def publish_marker(self, x, y, z, frame_id, tag_id):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "tags"
        marker.id = abs(hash(tag_id)) % 10000
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.01
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.lifetime.sec = 3
        self.marker_pub.publish(marker)

        # Text marker
        text_marker = Marker()
        text_marker.header.frame_id = frame_id
        text_marker.header.stamp = marker.header.stamp
        text_marker.ns = "tag_labels"
        text_marker.id = marker.id + 5000
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = x
        text_marker.pose.position.y = y
        text_marker.pose.position.z = z + 0.05
        text_marker.pose.orientation.w = 1.0
        text_marker.scale.z = 0.04
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 1.0
        text_marker.color.a = 1.0
        text_marker.text = tag_id
        text_marker.lifetime.sec = 3
        self.marker_pub.publish(text_marker)

def main(args=None):
    rclpy.init(args=args)
    node = TagDetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

