#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    my_first_pkg = get_package_share_directory('myfirstpackage')
    return LaunchDescription([
        Node(
            package='turtlesim',
            node_executable='turtlesim_node',
            node_name='my_turtle',
            parameters=[os.path.join(my_first_pkg, 'turtlesim_random.yaml')],
            node_namespace='new_turtle',
            output='screen'),
        Node(
            package='turtlesim',
            node_executable='draw_square',
            node_name='draw_square',
            node_namespace='new_turtle',
            output='log'),
    ])