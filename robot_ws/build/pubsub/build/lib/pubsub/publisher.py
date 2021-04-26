#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String

def main():
    global mypub
    rclpy.init()
    myfirstpublisher = rclpy.create_node('myfirstpublisher')
    mypub = myfirstpublisher.create_publisher(String, 'myfirsttopic', 1)
    myfirstpublisher.create_timer(0.1, mytimercallback)
    try:
        rclpy.spin(myfirstpublisher)
    except KeyboardInterrupt:
        pass

    myfirstpublisher.destroy_node()
    rclpy.shutdown()

def mytimercallback():
    global mypub
    mymsg = String()
    mymsg.data = 'Hello ROS2 Communication'
    mypub.publish(mymsg)

if __name__ == '__main__':
    main()
