import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')
    my_robot_slam = get_package_share_directory('my_robot_slam')
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo, 'launch/turtlebot3_house.launch.py')
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(my_robot_slam, 'localization.launch.py')
            ),
        ),
        Node(
            package='rqt_joy',
            executable='rqt_joy',
            name='rqt_joy',
            output='screen'),
        Node(
            package='tb3_teleop',
            executable='teleop',
            name='teleop',
            output='screen'),
        Node(
            package='my_robot_slam',
            executable='current_pose',
            name='current_pose',
            output='screen'), 
    ])