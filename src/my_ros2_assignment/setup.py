from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'my_ros2_assignment'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'scripts'), glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS2 두산 E0509 로봇암 시뮬레이션 제어 GUI',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'robot_gui = my_ros2_assignment.main_node:main',
        ],
    },
)
