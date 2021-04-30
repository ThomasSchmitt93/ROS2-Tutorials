#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='parent_to_child_tf',
            node_namespace='base_footprint_to_base_link',
            arguments=['-1.0', '0.0', '0.01', '0.0', '0.0', '0.0', 'base_footprint', 'my_base_link'],
            output='screen'),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='parent_to_child_tf',
            node_namespace='base_link_to_camera',
            arguments=['0.015', '0.0913', '0.0945', '0.0', '0.0', '0.0', 'my_base_link', 'my_camera_rgb'],
            output='screen'),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='parent_to_child_tf',
            node_namespace='base_link_to_imu',
            arguments=['0.0', '0.0', '0.068', '0.0', '0.0', '0.0', 'my_base_link', 'my_imu_link'],
            output='screen'),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='parent_to_child_tf',
            node_namespace='base_link_to_scan',
            arguments=['-0.09', '0.0', '0.114', '0.0', '0.0', '0.0', 'my_base_link', 'my_base_scan'],
            output='screen'),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='parent_to_child_tf',
            node_namespace='base_link_to_pole',
            arguments=['-0.22', '0.06', '0.038', '0.0', '0.0', '0.0', 'my_base_link', 'pole_link'],
            output='screen'),
        Node(
            package='tf2_ros',
            node_executable='static_transform_publisher',
            node_name='parent_to_child_tf',
            node_namespace='pole_link_to_flag',
            arguments=['0.0', '0.0', '0.02', '0.0', '0.0', '0.0', 'pole_link', 'flag_link'],
            output='screen'),
    ])