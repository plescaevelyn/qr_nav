import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os
import math
import time
from tf_transformations import euler_from_quaternion

class MarkerLogger(Node):
    def __init__(self):
        super().__init__('marker_logger_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            'tag_detections',
            self.listener_callback,
            10
        )

        logs_dir = os.path.expanduser('~/qr_nav_ws/logs')
        os.makedirs(logs_dir, exist_ok=True)
        self.csv_path = os.path.join(logs_dir, 'experimente_coduri.csv')

        if not os.path.isfile(self.csv_path):
            with open(self.csv_path, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    "tip_marker", "id", "distanta_m", "unghi_grade",
                    "iluminare", "detectat", "pozitie_x", "pozitie_y",
                    "pozitie_z", "observatii"
                ])

        self.last_detection_time = {}
        self.detection_cooldown = 2.0
        self.get_logger().info("MarkerLogger is active")

    def listener_callback(self, msg: PoseStamped):
        try:
            # Coordonates
            x = round(msg.pose.position.x, 3)
            y = round(msg.pose.position.y, 3)
            z = round(msg.pose.position.z, 3)
            dist = round((x**2 + y**2 + z**2) ** 0.5, 3)

            # Yaw extracted from quaternion
            q = msg.pose.orientation
            _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
            angle_deg = round(math.degrees(yaw), 1)

            # ID & frames extracted from frame_id
            tag_id_full = msg.header.frame_id  # ex: 'qr_MESAJ' or 'aruco_2'
            if "_" in tag_id_full:
                tip_marker, marker_id = tag_id_full.split("_", 1)
            else:
                tip_marker, marker_id = "unknown", tag_id_full

            # Debounce
            key = (tag_id_full,)
            now = time.time()
            if now - self.last_detection_time.get(key, 0) < self.detection_cooldown:
                return
            self.last_detection_time[key] = now

            row = [
                tip_marker,
                marker_id,
                dist,
                angle_deg,
                "medie",       # placeholder
                "da",
                x, y, z,
                "auto"
            ]

            with open(self.csv_path, mode='a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(row)

            self.get_logger().info(f"[LOG] {tag_id_full}: dist={dist}, angle={angle_deg}, pos=({x}, {y}, {z})")

        except Exception as e:
            self.get_logger().error(f"Error occured during message processing: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
