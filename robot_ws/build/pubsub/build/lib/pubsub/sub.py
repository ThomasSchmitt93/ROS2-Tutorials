#!/usr/bin/env python3

import rclpy
from std_msgs.msg import String

def main():
    rclpy.init()
    myfirstsubscriber = rclpy.create_node('myfirstsubscriber')
    myfirstsubscriber.create_subscription(String, 'myfirsttopic', mysubcallback, 10)
    try:
        rclpy.spin(myfirstsubscriber)
    except KeyboardInterrupt:
        pass
    myfirstsubscriber.destroy_node()
    rclpy.shutdown()

def mysubcallback(msg):
    print('You said: ', msg.data)

if __name__ == '__main__':
    main()