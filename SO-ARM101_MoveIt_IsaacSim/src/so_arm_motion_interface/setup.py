import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'so_arm_motion_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'websockets'],
    zip_safe=True,
    maintainer='teamgrit',
    maintainer_email='jhs10429@teamgrit.kr',
    description='Pose-driven MoveIt interface for the SO-ARM101 simulation',
    license='MIT',
    entry_points={
        'console_scripts': [
            'twist_to_servo_node = so_arm_motion_interface.twist_to_servo_node:main',
            'pose_to_servo_node = so_arm_motion_interface.pose_to_servo_node:main',
            'websocket_pose_bridge_node = so_arm_motion_interface.websocket_pose_bridge_node:main',
                    ],
    },
)
