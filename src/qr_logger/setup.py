from setuptools import setup

package_name = 'qr_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
	('share/' + package_name + '/launch', ['launch/detect_apriltags.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='evelyn',
    maintainer_email='evelynplesca@gmail.com',
    description='Nod ROS 2 care salvează detecții coduri vizuale în CSV',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'log_node = qr_logger.log_node:main',
        ],
    },
)

