from setuptools import setup
import os
from glob import glob

package_name = 'simple_autonomous_robot'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='A simple autonomous robot with LIDAR-based navigation and obstacle avoidance.',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_sensor = simple_autonomous_robot.lidar_sensor:main',
            'robot_controller = simple_autonomous_robot.robot_controller:main',
        ],
    },
)
