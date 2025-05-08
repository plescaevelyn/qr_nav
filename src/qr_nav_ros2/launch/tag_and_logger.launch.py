from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('depthai_ros_driver'), 'launch', 'camera.launch.py')
        ])
    )

    tag_detector_node = Node(
        package='qr_nav_ros2',
        executable='tag_detector',
        name='tag_detector',
        output='screen'
    )

    logger_node = Node(
        package='qr_logger',
        executable='logger_node',
        name='logger_node',
        output='screen'
    )

    rviz_config_path = os.path.join(
        get_package_share_directory('qr_nav_ros2'),
        'rviz',
        'qr_nav_testing.rviz'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        depthai_launch,
        tag_detector_node,
        logger_node,
        rviz_node
    ])

