#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from crazyflie_py import Crazyswarm
from crazyflie_interfaces.msg import FullStateArray, FullState, Hover
from tf2_msgs.msg import TFMessage
import numpy as np
from crazyflie_interfaces.msg import FullStateArray, FullState, LogDataGeneric
import yaml

class Logging_for_sim(Node):

    def __init__(self):
        super().__init__('sim_logger')
        
        # subscriber listens to the 250 Hz pose messages giving the poses of all the cfs
        self.pose_subscriber = self.create_subscription(
            TFMessage,
            'tf',
            self.pose_callback,
            10)
        
        # open config file and extract active cf_names
        with open('/coverage_crazyflie_ws/src/crazyswarm2/crazyflie/config/crazyflies.yaml', 'r') as file:
            config = yaml.safe_load(file)
        self.cf_names = ([cf_name for cf_name in list(filter(lambda x: config['robots'][x]['enabled'],config['robots'].keys()))])
        self.posesLists = {}
        for cf_name in self.cf_names:
            self.posesLists[cf_name] = []
        
        # sends poses at given frequency
        self.pubList = {}
        for cf_name in self.cf_names:
            self.pubList[cf_name] = self.create_publisher(LogDataGeneric, '/'+ cf_name+'/posVelAcc', 1)
        self.timer = self.create_timer(config['all']['firmware_logging']['custom_topics']['posVelAcc']['frequency']**(-1), self.log_poses)
        
    
    def pose_callback(self, msg):
        for transform in msg.transforms:
            cf_name = transform.child_frame_id
            if(len(self.posesLists[cf_name]) == 3):
                self.posesLists[cf_name].pop(0)
            T = transform.transform.translation
            time = transform.header.stamp.sec + msg.transforms[0].header.stamp.nanosec * 1e-9
            self.posesLists[cf_name].append(np.array([time, T.x, T.y, T.z]))
        
    def log_poses(self):
        for cf_name in self.cf_names:
            posesList = self.posesLists[cf_name]
            if(len(posesList) == 3):
                msg = LogDataGeneric()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = cf_name
                msg.values = [0.0]*6
                # xy positions
                msg.values[0] = posesList[2][1]*1000
                msg.values[1] = posesList[2][2]*1000
                # xy speeds
                speeds21 = posesList[2] - posesList[1]
                speeds21 = speeds21[1:3]/speeds21[0]
                msg.values[2] = speeds21[0]*1000
                msg.values[3] = speeds21[1]*1000
                
                # xy accelerations
                speeds10 = posesList[1] - posesList[0]
                speeds10 = speeds10[1:3]/speeds10[0]
                accs = 0.5*(speeds21 - speeds10)/(posesList[2][0] - posesList[0][0])
                msg.values[4] = accs[0]*1000
                msg.values[5] = accs[1]*1000
                self.pubList[cf_name].publish(msg)
    
def main(args=None):
    rclpy.init(args=args)
    logging_for_sim = Logging_for_sim()
    rclpy.spin(logging_for_sim)
    logging_for_sim.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()