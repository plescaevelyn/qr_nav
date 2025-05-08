from setuptools import setup

package_name = 'qr_nav_ros2'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
    	('share/ament_index/resource_index/packages', ['resource/qr_nav_ros2']),
    	('share/qr_nav_ros2', ['package.xml']),
    	('share/qr_nav_ros2/launch', ['launch/tag_and_logger.launch.py']),
    	('share/qr_nav_ros2/rviz', ['rviz/qr_nav_testing.rviz']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evelyn',
    maintainer_email='evelynplesca@gmail.com',
    description='QR/AprilTag detection and navigation',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tag_detector = qr_nav_ros2.tag_detector:main',
        ],
    },
)

