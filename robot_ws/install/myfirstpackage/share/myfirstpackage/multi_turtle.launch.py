#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    my_first_pkg = get_package_share_directory('myfirstpackage')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_first_pkg, 'turtle.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_first_pkg, 'turtle2.launch.py')
            ),
        ),
        Node(
            package='myfirstpackage',
            node_executable='myfirstnode',
            node_name='myfirstnode',
            output='log'),
    ])
