#!/usr/bin/env python3

import rclpy
from time import sleep

def main():
    rclpy.init()
    myfirstnode = rclpy.create_node('my_param_node')
    myfirstnode.declare_parameter('my_param', 13)
    while rclpy.ok():
        try:
            rclpy.spin_once(myfirstnode, timeout_sec=1.0)
        except KeyboardInterrupt:
            pass
        print(myfirstnode.get_parameter('my_param').value)
        sleep(1.0)
    myfirstnode.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()