from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='apriltag_ros',
            executable='apriltag_node',
            name='apriltag_node',
            parameters=[{
                'publish_tag_detections_image': True,
                'camera_frame': 'oak_rgb_camera_frame'
            }],
            remappings=[
                ('/image_rect', '/oak/rgb/image_raw'),
                ('/camera_info', '/oak/rgb/camera_info'),
		('/tag_detections', '/apriltag_detections') 
            ]
        )
    ])
