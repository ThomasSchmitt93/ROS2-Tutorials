#!/usr/bin/env python3

import rclpy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

direction = Twist()

def main():
    global mypub
    rclpy.init()
    teleop = rclpy.create_node('teleop')
    teleop.create_subscription(Joy, '/joy', mysubcallback, 10)
    mypub = teleop.create_publisher(Twist, '/cmd_vel', 1)
    try:
        rclpy.spin(teleop)
    except KeyboardInterrupt:
        pass
    teleop.destroy_node()
    rclpy.shutdown()

def mysubcallback(msg):
    global direction
    global mypub
    direction.linear.x = msg.axes[0]
    direction.angular.z = msg.axes[1]

    mypub.publish(direction)

if __name__ == '__main__':
    main()