import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import csv
import os
from datetime import datetime

class MarkerLogger(Node):
    def __init__(self):
        super().__init__('marker_logger_node')
        self.subscription = self.create_subscription(
            PoseStamped,
            '/tag_detections',
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

        self.get_logger().info("MarkerLogger activ")

    def listener_callback(self, msg):
        x = round(msg.pose.position.x, 3)
        y = round(msg.pose.position.y, 3)
        z = round(msg.pose.position.z, 3)
        dist = round((x**2 + y**2 + z**2) ** 0.5, 3)

        row = [
            "apriltag",
            0,              # marker ID
            dist,
            0,              # unknown angle from PoseStamped
            "medie",        
            "da",
            x,
            y,
            z,
            "auto"
        ]

        with open(self.csv_path, mode='a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(row)

        self.get_logger().info(f"Logged: x={x}, y={y}, z={z}, dist={dist}")

def main(args=None):
    rclpy.init(args=args)
    node = MarkerLogger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
