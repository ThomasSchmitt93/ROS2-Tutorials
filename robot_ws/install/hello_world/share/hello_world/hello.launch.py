#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    my_first_pkg = get_package_share_directory('hello_world')
    return LaunchDescription([
        Node(
            package='hello_world',
            node_executable='hello_world',
            node_name='hello_world',
            parameters=[os.path.join(my_first_pkg, 'hello.yaml')],
            output='screen',
            emulate_tty=True),
    ])