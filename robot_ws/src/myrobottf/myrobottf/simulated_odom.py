#!/usr/bin/env python3

import rclpy
import tf2_ros

import scipy
import numpy
from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation

cnt = 0

def calculate_position():
    global cnt
    if cnt > 300:
        cnt = 0
    cnt = cnt+1
    x = 3* numpy.cos(cnt*0.02)
    y = numpy.sin(2*cnt*0.02)

    position = (x,y)
    return position

def calculate_tf():
    global mydynamictf
    t = TransformStamped()
    #getting actual position
    position_turtlebot = calculate_position()
    #calculating transform
    now = mydynamictf.get_clock().now().to_msg()
    t.header.stamp = now
    t.header.frame_id = "odom"
    t.child_frame_id = "base_footprint"
    t.transform.translation.x = position_turtlebot[0]
    t.transform.translation.y = position_turtlebot[1]
    t.transform.translation.z = 0.0
    euler = Rotation.from_euler('zyx', [0.0, 0.0, 0.0])
    quat = euler.as_quat()
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t

def mytimercallback():
    global mydynamictf
    odom_to_base_footprint = calculate_tf()
    br = tf2_ros.transform_broadcaster.TransformBroadcaster(mydynamictf)
    br.sendTransform(odom_to_base_footprint)

def main():
    global mydynamictf
    rclpy.init()
    mydynamictf = rclpy.create_node('my_dynamic_tf')
    mydynamictf.create_timer(1.0 / 30.0, mytimercallback)
    try:
        rclpy.spin(mydynamictf)
    except KeyboardInterrupt:
        pass    
    mydynamictf.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()