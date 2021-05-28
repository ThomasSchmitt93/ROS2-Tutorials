# geometry_msgs.msg.TransformStamped(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=1621607295, nanosec=101333777), frame_id='camera'),
# child_frame_id='marker', transform=geometry_msgs.msg.Transform
# (translation=geometry_msgs.msg.Vector3(x=0.04018274184199045, y=-0.007529640862626247, z=0.11799290006129155),
# rotation=geometry_msgs.msg.Quaternion(x=0.9984696075390636, y=0.04832304375979252, z=-0.026306677109187285, w=0.005593299746169032)))

#!/usr/bin/env python3

import rclpy
import tf2_ros

import math
import numpy
import sys
import termios

from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import QoSProfile
from nav_msgs.msg import Odometry

from turtlebot3_example.turtlebot3_position_control.turtlebot3_path import Turtlebot3Path

def timercallback():
    global tfBuffer
    try:
        # do a lookup transform between 'base_link' and 'marker' frame
        trans = tfBuffer.lookup_transform("camera", "marker", rclpy.duration.Duration())
        # returns TransformStamped() message
        print(trans) # print lookup transform
        x_trans = trans.x
        print(x_trans)
    except:
        # exception is raised on extrapolation, 
        # no connection between frames or when frames dont exist
        print("lookup failed!") 

def main():
    global tfBuffer
    rclpy.init() # init ros client library
    nh = rclpy.create_node('tf2_listener') # create a node with name 'tf2_listener'
    tfBuffer = tf2_ros.Buffer() # create a TF2 buffer which saves the TFs for given cache_time
    tf2_ros.TransformListener(tfBuffer, nh) # create TF2 listener which connects buffer with node
    nh.create_timer(0.1, timercallback) # call timercallback every 100ms
    try:
        rclpy.spin(nh) # spin node until exception
    except KeyboardInterrupt:
        nh.destroy_node() # destroy node
        rclpy.shutdown() # shutdown ros client library

if __name__ == '__main__':
    main()








# class Turtlebot3PositionControl(Node):

#     def __init__(self):
#         super().__init__('turtlebot3_position_control')

#         """************************************************************
#         ** Initialise variables
#         ************************************************************"""
#         self.odom = Odometry()
#         self.last_pose_x = 0.0
#         self.last_pose_y = 0.0
#         self.last_pose_theta = 0.0
#         self.goal_pose_x = 0.0
#         self.goal_pose_y = 0.0
#         self.goal_pose_theta = 0.0
#         self.step = 1
#         self.get_key_state = False
#         self.init_odom_state = False  # To get the initial pose at the beginning

#         """************************************************************
#         ** Initialise ROS publishers and subscribers
#         ************************************************************"""
#         qos = QoSProfile(depth=10)

#         # Initialise publishers
#         self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', qos)

#         # Initialise subscribers
#         self.odom_sub = self.create_subscription(
#             Odometry,
#             'odom',
#             self.odom_callback,
#             qos)

#         """************************************************************
#         ** Initialise timers
#         ************************************************************"""
#         self.update_timer = self.create_timer(0.010, self.update_callback)  # unit: s

#         self.get_logger().info("Turtlebot3 position control node has been initialised.")

#     """*******************************************************************************
#     ** Callback functions and relevant functions
#     *******************************************************************************"""
#     def odom_callback(self, msg):
#         self.last_pose_x = msg.pose.pose.position.x
#         self.last_pose_y = msg.pose.pose.position.y
#         _, _, self.last_pose_theta = self.euler_from_quaternion(msg.pose.pose.orientation)

#         self.init_odom_state = True

#     def update_callback(self):
#         if self.init_odom_state is True:
#             self.generate_path()

#     def generate_path(self):
#         twist = Twist()

#         if self.get_key_state is False:
#             input_x, input_y, input_theta = self.get_key()
#             self.goal_pose_x = self.last_pose_x + input_x
#             self.goal_pose_y = self.last_pose_y + input_y
#             self.goal_pose_theta = self.last_pose_theta + input_theta

#         else:
#             # Step 1: Turn
#             if self.step == 1:
#                 path_theta = math.atan2(
#                     self.goal_pose_y - self.last_pose_y,
#                     self.goal_pose_x - self.last_pose_x)
#                 angle = path_theta - self.last_pose_theta
#                 angular_velocity = 0.1  # unit: rad/s

#                 twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

#             # Step 2: Go Straight
#             elif self.step == 2:
#                 distance = math.sqrt(
#                     (self.goal_pose_x - self.last_pose_x)**2 +
#                     (self.goal_pose_y - self.last_pose_y)**2)
#                 linear_velocity = 0.1  # unit: m/s

#                 twist, self.step = Turtlebot3Path.go_straight(distance, linear_velocity, self.step)

#             # Step 3: Turn
#             elif self.step == 3:
#                 angle = self.goal_pose_theta - self.last_pose_theta
#                 angular_velocity = 0.1  # unit: rad/s

#                 twist, self.step = Turtlebot3Path.turn(angle, angular_velocity, self.step)

#             # Reset
#             elif self.step == 4:
#                 self.step = 1
#                 self.get_key_state = False

#             self.cmd_vel_pub.publish(twist)

#     def get_key(self):
#         # Print terminal message and get inputs
#         input_x = float(input("Input x: "))
#         input_y = float(input("Input y: "))
#         input_theta = float(input("Input theta: "))
#         while input_theta > 180 or input_theta < -180:
#             self.get_logger().info("Enter a value for theta between -180 and 180")
#             input_theta = input("Input theta: ")
#         input_theta = numpy.deg2rad(input_theta)  # Convert [deg] to [rad]

#         settings = termios.tcgetattr(sys.stdin)
#         termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

#         return input_x, input_y, input_theta

#     """*******************************************************************************
#     ** Below should be replaced when porting for ROS 2 Python tf_conversions is done.
#     *******************************************************************************"""
#     def euler_from_quaternion(self, quat):
#         """
#         Convert quaternion (w in last place) to euler roll, pitch, yaw.
#         quat = [x, y, z, w]
#         """
#         x = quat.x
#         y = quat.y
#         z = quat.z
#         w = quat.w

#         sinr_cosp = 2 * (w * x + y * z)
#         cosr_cosp = 1 - 2 * (x * x + y * y)
#         roll = numpy.arctan2(sinr_cosp, cosr_cosp)

#         sinp = 2 * (w * y - z * x)
#         pitch = numpy.arcsin(sinp)

#         siny_cosp = 2 * (w * z + x * y)
#         cosy_cosp = 1 - 2 * (y * y + z * z)
#         yaw = numpy.arctan2(siny_cosp, cosy_cosp)

#         return roll, pitch, yaw