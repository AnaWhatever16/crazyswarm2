#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, PoseArray, Pose
from crazyflie_py import Crazyswarm
from crazyflie_interfaces.srv import Takeoff
import numpy as np

import time

def position(cf, pos):
    vel = np.array([0.0, 0.0, 0.0])
    acc = np.array([0.0, 0.0, 0.0])
    yaw = 0.0
    omega = np.array([0.0, 0.0, 0.0])
    cf.cmdFullState(pos, vel, acc, yaw, omega)

class Target_executor(Node):
    def __init__(self):
        super().__init__('target_executor')
        
        self.takeoff_cli = self.create_client(Takeoff, '/all/takeoff')
        self.req = Takeoff.Request()
        
        # self.subscriber_ = self.create_subscription(
        #     PoseArray,
        #     'target',
        #     self.target_executor_callback,
        #     10)

    def send_request(self, height, duration):
        self.req.height = height
        self.req.duration.sec = int(duration)
        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
    
    def target_executor_callback(self, msg):
        print([[i.x, i.y, i.z] for i in msg.poses])

def main(args=None):
    swarm = Crazyswarm()
    print(swarm.allcfs.__dict__.keys())
    rclpy.init(args=args)

    target_executor = Target_executor()

    rclpy.spin(target_executor)

    target_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
