#!/usr/bin/env python3

from nav2_msgs.action import FollowWaypoints
from geometry_msgs.msg import PoseStamped

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

import math
import numpy as np

class MinimalActionClient(Node):

    def __init__(self):
        super().__init__('minimal_action_client')
        self._client = ActionClient(self, FollowWaypoints, '/FollowWaypoints')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            self.send_points(mgoal)
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback        
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_waypoint))

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.missed_waypoints))
        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_points(self, points):
        # self.get_logger().info('Waiting for action server...')
        
        msg = FollowWaypoints.Goal()
        msg.poses = points
        
        self._client.wait_for_server()
        self._send_goal_future = self._client.send_goal_async(msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)    
        self.get_logger().info('Sending goal request...')

def genpoints(x1, y1, x2, y2): #Send 2 diagonal cornerpoints of the room

    w = 0.5 #distance between 2 lanes
    nr_of_lanes = math.trunc((x2-x1)/w + 1)
    nr_of_waypoints = 2*nr_of_lanes
    points = np.empty([nr_of_waypoints, 2])

    j = 1; #auxiliary loop index
    k = 0; #auxiliary loop index

    for i in range(nr_of_waypoints):
        if j == 1:
            points[i,0] = x1 + (k*w)
            points[i,1] = y1
        elif j == 2:
            points[i,0] = x1 + (k*w)
            points[i,1] = y2
            k = k + 1
        elif j == 3:
            points[i,0] = x1 + (k*w)
            points[i,1] = y2    
        elif j == 4:
            points[i,0] = x1 + (k*w)
            points[i,1] = y1
            k = k + 1
        j = j + 1

        if j == 5:
            j = 1
    return(points)

def main(args=None):
    global mgoal
    rclpy.init(args=args)
    action_client = MinimalActionClient()
    
    points1 = genpoints(-3.7, 4.6, -0.7, 0.9)
    points2 = genpoints(-6.5, 4.4, -5.7, 2.0)
    points3 = genpoints(-6.7, 0.5, -5.7, -2.9)
    points4 = genpoints(5.6, -0.5, 6.8, -4.2)
    points5 = genpoints(3.0, 4.7, 6.8, 0.7)
    points6 = genpoints(0.6, 4.8, 1.6, 2.8)
    points7 = genpoints(0.6, 2.8, 1.3, 1.7)
    waypoints = np.append(points1, points2, axis = 0)
    waypoints = np.append(waypoints, points3, axis = 0)
    waypoints = np.append(waypoints, points4, axis = 0)
    waypoints = np.append(waypoints, points5, axis = 0)
    waypoints = np.append(waypoints, points6, axis = 0)
    waypoints = np.append(waypoints, points7, axis = 0)
    mgoal = []

    for i in range(np.size(waypoints, 0)):
        waypoint = PoseStamped()
        waypoint.header.frame_id = "map"
        waypoint.pose.position.x =  waypoints[i,0]
        waypoint.pose.position.y =  waypoints[i,1]
        mgoal.append(waypoint)

    action_client.send_points(mgoal)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()