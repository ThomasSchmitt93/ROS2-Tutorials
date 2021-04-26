#!/usr/bin/env python3

import rclpy
from std_srvs.srv import SetBool
import sys

from rclpy.node import Node



class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = self.create_client(SetBool, 'open_gripper')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = SetBool.Request()

    def send_openrequest(self):
        self.req.data = False
        self.future = self.cli.call_async(self.req)

    def send_closerequest(self):
        self.req.data = True
        self.future = self.cli.call_async(self.req)


def main(args=None):
    rclpy.init(args=args)

    minimal_client = MinimalClientAsync()
    minimal_client.send_openrequest()

    while rclpy.ok():
        rclpy.spin_once(minimal_client)
        if minimal_client.future.done():
            try:
                response = minimal_client.future.result()
            except Exception as e:
                minimal_client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                if response.success == False:
                    minimal_client.send_closerequest()

            break

    minimal_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()