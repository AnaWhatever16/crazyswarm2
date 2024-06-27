#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PolygonStamped, PoseArray, Pose
from tf2_msgs.msg import TFMessage
import numpy as np
from crazyflie_interfaces.msg import FullStateArray, FullState, LogDataGeneric


import time

from numpy import *
import matplotlib.pyplot as plt
import os

data = []

class Algorithm(Node):
    def __init__(self):
        self.polygon = np.array([[1,1],[-1,1],[-1,-1],[1,-1]])*0.9
        self.startTime = time.time()
        
        super().__init__('algorithm')
        self.polygon_subscriber = self.create_subscription(
            PolygonStamped,
            'polygon_ver',
            self.polygon_callback,
            10)
        self.pose_subscriber = self.create_subscription(
            LogDataGeneric,
            '/cf2222/speed',
            self.pose_callback,
            10)
        self.target_publisher = self.create_publisher(FullStateArray, 'target', 10)
    
    def polygon_callback(self, msg):
        self.polygon = [[i.x,i.y] for i in msg.polygon.points]
        
    def pose_callback(self, msg):
        data.append([msg.header.stamp.sec + msg.header.stamp.nanosec*1e-9 -self.startTime] + list(msg.values))
        print(data[-1])
        target_msg = FullStateArray()
        
        target_msg.fullstates.append(FullState())
        target_msg.fullstates[-1].pose.position.z = 1.0
        target_msg.fullstates[-1].twist.linear.z = 0.0
        target_msg.fullstates[-1].twist.linear.x = 100.0
        
        target_msg.fullstates[-1].acc.x = 0.5 if data[-1][0]>1 and data[-1][0]<3  else 0.0
        target_msg.fullstates[-1].acc.y = 0.0 
        target_msg.fullstates[-1].acc.z = 0.0
            
        self.target_publisher.publish(target_msg)
        

def main(args=None):
    try:
        rclpy.init(args=args)

        algorithm = Algorithm()

        rclpy.spin(algorithm)

        algorithm.destroy_node()
        rclpy.shutdown()
    except KeyboardInterrupt:        
        TIME = time.localtime()
        TIME = str(TIME.tm_year) + "_" + str(TIME.tm_mon) + "_" + str(TIME.tm_mday) + "_" + str(TIME.tm_hour) + "h" + str(TIME.tm_min) + "m" + str(TIME.tm_sec) + "s"
        path = os.path.join("/coverage_crazyflie_ws/src/crazyswarm2/crazyflie/results", TIME) 
        os.mkdir(path)
        np.savetxt(os.path.join(path,'amorti.txt'),array(data)/1000)
    
if __name__ == '__main__':
    main()
