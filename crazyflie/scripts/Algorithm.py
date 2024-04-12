#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped, PoseArray, Pose

import time

class Algorithm(Node):
    def __init__(self):
        super().__init__('algorithm')
        self.polygon_subscriber = self.create_subscription(
            PolygonStamped,
            'polygon_ver',
            self.polygon_callback,
            10)
        self.target_publisher = self.create_publisher(PoseArray, 'target', 10)
        

    def polygon_callback(self, msg):
        points = [[i.x,i.y] for i in msg.polygon.points]
        #print(points)
        target_msg = PoseArray()
        target_msg.header.frame_id = 'world'
        for point in points:
            target_msg.poses.append(Pose())
            target_msg.poses[-1].position.x = float(point[0])
            target_msg.poses[-1].position.y = float(point[1])
            target_msg.poses[-1].position.z = 1.0
            target_msg.poses[-1].orientation.x = 0.0
            target_msg.poses[-1].orientation.y = 0.0
            target_msg.poses[-1].orientation.z = 0.0
            target_msg.poses[-1].orientation.w = 1.0
        self.target_publisher.publish(target_msg)

def main(args=None):
    rclpy.init(args=args)

    algorithm = Algorithm()

    rclpy.spin(algorithm)

    algorithm.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
