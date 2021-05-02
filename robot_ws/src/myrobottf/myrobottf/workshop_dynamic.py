#!/usr/bin/env python3

import rclpy
import tf2_ros

import scipy
import numpy
from geometry_msgs.msg import TransformStamped

from scipy.spatial.transform import Rotation

def calculate_tf():
    global mydynamictf
    t = TransformStamped()

    #calculating transform
    now = mydynamictf.get_clock().now().to_msg()
    t.header.stamp = now
    t.header.frame_id = "my_base_link"
    t.child_frame_id = "pole_link"
    t.transform.translation.x = -0.22
    t.transform.translation.y = 0.06
    t.transform.translation.z = 0.038
    euler = Rotation.from_euler('zyx', [0.0, 0.0, 0.0])
    quat = euler.as_quat()
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t

def calculate_tf2():
    global mydynamictf
    t = TransformStamped()

    #calculating transform
    now = mydynamictf.get_clock().now().to_msg()
    t.header.stamp = now
    t.header.frame_id = "pole_link"
    t.child_frame_id = "flag_link"
    t.transform.translation.x = 0.0
    t.transform.translation.y = 0.0
    t.transform.translation.z = 0.02
    euler = Rotation.from_euler('zyx', [0.0, 0.0, 0.0])
    quat = euler.as_quat()
    t.transform.rotation.x = quat[0]
    t.transform.rotation.y = quat[1]
    t.transform.rotation.z = quat[2]
    t.transform.rotation.w = quat[3]
    return t

def mytimercallback():
    global mydynamictf
    pole_link_to_my_base_link = calculate_tf()
    br = tf2_ros.transform_broadcaster.TransformBroadcaster(mydynamictf)
    br.sendTransform(pole_link_to_my_base_link)

    flag_link_to_pole_link = calculate_tf2()
    br2 = tf2_ros.transform_broadcaster.TransformBroadcaster(mydynamictf)
    br2.sendTransform(flag_link_to_pole_link)

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