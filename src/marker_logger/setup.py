from setuptools import setup

package_name = 'marker_logger'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adi',
    maintainer_email='you@example.com',
    description='Logger and QR/ArUco/AprilTag code analysis',
    license='MIT',
    entry_points={
        'console_scripts': [
            'logger_node = marker_logger.marker_logger:main'
        ],
    },
)

