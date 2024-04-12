#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from crazyflie_py import Crazyswarm

class Target_executor(Node):

    def __init__(self):
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        super().__init__('target_executor')
        self.get_logger().info('heeeeeeelp1')
        self.swarm.allcfs.takeoff(targetHeight=1.0, duration=1.0)
        self.timeHelper.sleep(2.0)
    
        # Création d'un abonné pour écouter les positions
        self.target_sub = self.create_subscription(
            PoseArray,
            'target',
            self.position_callback,
            10)

    def position_callback(self, msg):
        positions = [[i.position.x, i.position.y, i.position.z] for i in msg.poses]
        for i, cf in enumerate(self.swarm.allcfs.crazyflies):
            self.position(cf, positions[i])

    def position(self, cf, pos):
        vel = [0.0, 0.0, 0.0]
        acc = [0.0, 0.0, 0.0]
        yaw = 0.0
        omega = [0.0, 0.0, 0.0]
        cf.cmdFullState(pos, vel, acc, yaw, omega)

def main(args=None):
    # rclpy.init(args=args)
    target_executor = Target_executor()
    rclpy.spin(target_executor)
    target_executor.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()