#!/usr/bin/env python3

import rclpy
import tf2_ros
from std_msgs.msg import String

def timercallback():
    global tfBuffer
    global current_pos_pub
    try:
        # do a lookup transform between 'base_link' and 'marker' frame
        trans = tfBuffer.lookup_transform("map", "base_footprint", rclpy.duration.Duration())
        # returns TransformStamped() message
        trans_x = trans.transform.translation.x
        trans_y = trans.transform.translation.y
        rot_z = trans.transform.rotation.z
        mymsg = String()
        pub_string = 'Postion x = {}\nPostion y = {}\nRotation = {}\n'.format(trans_x,trans_y,rot_z)
        mymsg.data = pub_string
        current_pos_pub.publish(mymsg)
        #print(trans) # print lookup transform
    except:
        # exception is raised on extrapolation, 
        # no connection between frames or when frames dont exist
        print("lookup failed") 

def main():
    global tfBuffer
    global current_pos_pub
    rclpy.init() # init ros client library
    nh = rclpy.create_node('tf2_listener') # create a node with name 'tf2_listener'
    tfBuffer = tf2_ros.Buffer() # create a TF2 buffer which saves the TFs for given cache_time
    tf2_ros.TransformListener(tfBuffer, nh) # create TF2 listener which connects buffer with node
    current_pos_pub = nh.create_publisher(String, 'current_pose_2d', 1)
    nh.create_timer(0.1, timercallback) # call timercallback every 100ms
    try:
        rclpy.spin(nh) # spin node until exception
    except KeyboardInterrupt:
        nh.destroy_node() # destroy node
        rclpy.shutdown() # shutdown ros client library

if __name__ == '__main__':
    main()