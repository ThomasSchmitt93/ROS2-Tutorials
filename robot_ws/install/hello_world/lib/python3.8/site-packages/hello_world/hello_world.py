#!/usr/bin/env python3

import rclpy
from time import sleep

def main():
    rclpy.init()
    hello_world = rclpy.create_node('hello_world')
    hello_world.declare_parameter('text', 'Hello ROS2 World! ')
    hello_world.declare_parameter('freq', 1)
    counter = 0
    while rclpy.ok():
        print(hello_world.get_parameter('text').value + 'Number of prints: ' + str(counter))
        counter += 1
        try:
            rclpy.spin_once(hello_world, timeout_sec=1/(hello_world.get_parameter('freq').value))
        except KeyboardInterrupt:
            hello_world.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()