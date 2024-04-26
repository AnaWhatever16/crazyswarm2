#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from crazyflie_py import Crazyswarm
from crazyflie_interfaces.msg import FullStateArray, FullState

class Target_executor(Node):

    def __init__(self):
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        super().__init__('target_executor')
        self.swarm.allcfs.takeoff(targetHeight=1.0, duration=1.0)
        self.timeHelper.sleep(1.0)
        # self.swarm.allcfs.crazyflies[0].land(targetHeight=0.04, duration=4.0)
        # self.timeHelper.sleep(3.0)
    
        # Création d'un abonné pour écouter les positions
        self.target_sub = self.create_subscription(
            FullStateArray,
            'target',
            self.cmd_callback,
            10)

    def cmd_callback(self, msg):
        positions = [[i.pose.position.x, i.pose.position.y, i.pose.position.z] for i in msg.fullstates]
        speeds = [[i.twist.linear.x, i.twist.linear.y, i.twist.linear.z] for i in msg.fullstates]
        acc = [[i.acc.x, i.acc.y, i.acc.z] for i in msg.fullstates]
        for i, cf in enumerate(self.swarm.allcfs.crazyflies):
            self.position(cf, positions[i],speeds[i], acc[i])

    def position(self, cf, pos, vel, acc):
        yaw = 0.0; omega = [0.0, 0.0, 0.0]
        cf.cmdFullState(pos, vel, acc, yaw, omega)

def main(args=None):
    # rclpy.init(args=args)
    target_executor = Target_executor()
    rclpy.spin(target_executor)
    target_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()