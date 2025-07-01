from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'serial_imu_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='Serial IMU to ROS2 Bridge Package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'serial_imu_bridge = serial_imu_bridge.serial_imu_bridge:main',
        ],
    },
)
