import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'so_arm_custom_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='teamgrit',
    maintainer_email='jhs10429@teamgrit.kr',
    description='(Deprecated) Custom planar IK + joint control for SO-ARM101, fed by web Pose',
    license='MIT',
    entry_points={
        'console_scripts': [
            'lecabot_direct_controller_node = so_arm_custom_control.lecabot_direct_controller_node:main',
            'custom_direct_controller_node = so_arm_custom_control.lecabot_direct_controller_node:main',
        ],
    },
)
