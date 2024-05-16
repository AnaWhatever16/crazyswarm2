#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


from geometry_msgs.msg import PoseStamped

from motion_capture_tracking_interfaces.msg import NamedPoseArray, NamedPose
import numpy as np


class VRPN_to_poses(Node):
    def __init__(self):
        super().__init__('vrpn_to_poses')
        qos_profile_sub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, 
                                 history=HistoryPolicy.KEEP_LAST,
                                 depth=10)
        qos_profile_pub = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, 
                                 history=HistoryPolicy.KEEP_LAST,
                                 deadline=rclpy.time.Duration(seconds=0.01),
                                 depth=10)
        self.publisher_ = self.create_publisher(NamedPoseArray, '/poses', qos_profile_pub)
        names = ['cf0000','cf1111','cf2222']
        func = {'cf0000':self.func0,'cf1111':self.func1,'cf2222':self.func2}
        subscribers = [self.create_subscription(PoseStamped, '/vrpn_mocap/'+ name +'/pose', func[name], qos_profile_sub) for name in names]
        
        self.pose0 = NamedPose()
        self.pose1 = NamedPose()
        self.pose2 = NamedPose()
        
        self.timer = self.create_timer(0.02, self.publish_poseArray)
        
    def func0(self,msg):
        self.pose0 = NamedPose()
        self.pose0.name = 'cf0000'
        self.pose0.pose = msg.pose

    
    def func1(self,msg):
        self.pose1 = NamedPose()
        self.pose1.name = 'cf1111'
        self.pose1.pose = msg.pose
        
        
    def func2(self,msg):
        self.pose2 = NamedPose()
        self.pose2.name = 'cf2222'
        self.pose2.pose = msg.pose
        
    def publish_poseArray(self):
        msg = NamedPoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        msg.poses = [self.pose0, self.pose1,self.pose2]
        self.publisher_.publish(msg)
   
    

def main(args=None):
    rclpy.init(args=args)
    vrpn_to_poses = VRPN_to_poses()
    rclpy.spin(vrpn_to_poses)
    vrpn_to_poses.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()