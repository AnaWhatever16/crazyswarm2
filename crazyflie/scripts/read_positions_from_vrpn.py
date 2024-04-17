#!/usr/bin/env python3


import rclpy
from rclpy.node import Node

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from motion_capture_tracking_interfaces.msg import NamedPoseArray, NamedPose


class VRPN_to_poses(Node):
    def __init__(self):
        super().__init__('vrpn_to_poses')
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, 
                                 history=HistoryPolicy.KEEP_LAST,
                                 deadline=rclpy.time.Duration(seconds=0.01),
                                 depth=10)
        self.publisher_ = self.create_publisher(NamedPoseArray, '/poses', qos_profile)
        
        for name in ['cf0000','cf1111','cf2222']:
            def translator(posestamped):
                msg = NamedPoseArray()
                header = msg.header
                header.frame_id = "world"
                
                pose1 = NamedPose()
                pose1.name = name
                pose1.pose.position = posestamped.pose.position
                
                pose1.pose.orientation = posestamped.pose.orientation
                msg.poses = []
                msg.poses.append(pose1)
                
                self.publisher_.publish(msg)
                
            self.create_subscription(
                PoseStamped,
                "/vrpn_mocap/" + name + "/pose",
                translator,
                qos_profile)



class Poses_publisher(Node):
    N = 0
    def __init__(self):
        super().__init__('mocap_for_cf' + str(Poses_publisher.N))
        Poses_publisher.N += 1
        
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, 
                                 history=HistoryPolicy.KEEP_LAST,
                                 deadline=rclpy.time.Duration(seconds=0.01),
                                 depth=10)
        
        self.publisher_ = self.create_publisher(NamedPoseArray, '/poses', qos_profile)

    def send_message(self, posestamped, name):
        msg = NamedPoseArray()
        header = msg.header

        header.frame_id = "world"
        
        pose1 = NamedPose()
        pose1.name = name
        pose1.pose.position = posestamped.pose.position
        
        pose1.pose.orientation = posestamped.pose.orientation
        msg.poses = []
        msg.poses.append(pose1)
        
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % pose1.name)


class pose_subscription(Node):
    N = 0
    def __init__(self, publisher, cf):
        self.cf = cf
        super().__init__('subscriber' + str(pose_subscription.N))
        pose_subscription.N += 1
        
        qos_profile = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, 
                                 history=HistoryPolicy.KEEP_LAST,
                                 depth=10)
        self.subscription = self.create_subscription(
            PoseStamped,
            "/vrpn_mocap/" + cf + "/pose",
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.publisher = publisher

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%f, %f, %f"' % (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z) )
        self.publisher.send_message(msg, self.cf)

def main(args=None):
    rclpy.init(args=args)
    
    vrpn_to_poses = VRPN_to_poses()
    
    rclpy.spin(vrpn_to_poses)
    
    vrpn_to_poses.destroy_node()
    
    # translators = list(map(lambda x: pose_subscription(Poses_publisher(),x), ['cf0000', 'cf1111', 'cf2222']))
    # for translator in translators:
    #     rclpy.spin(translator)

    # for translator in translators:
    #     translator.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()