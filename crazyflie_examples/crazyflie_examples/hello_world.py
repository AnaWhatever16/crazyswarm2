#!/usr/bin/env python3

# """Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from crazyflie_py import Crazyswarm


# TAKEOFF_DURATION = 4.0
# HOVER_DURATION = 5.0



# def main():
#     swarm = Crazyswarm()
#     timeHelper = swarm.timeHelper
#     cf = swarm.allcfs.crazyflies[0]

#     cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
#     timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    
    
#     cf.land(targetHeight=0.04, duration=2.5)
#     timeHelper.sleep(TAKEOFF_DURATION)


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3

# """Takeoff-hover-land for one CF. Useful to validate hardware config."""

# from crazyflie_py import Crazyswarm
# import numpy as np
# from math import cos, sin, pi


# TAKEOFF_DURATION = 1.0
# HOVER_DURATION = 1.0

# def position(cf, pos):
#     vel = np.array([0.0, 0.0, 0.0])
#     acc = np.array([0.0, 0.0, 0.0])
#     yaw = 0.0
#     omega = np.array([0.0, 0.0, 0.0])
#     cf.cmdFullState(pos, vel, acc, yaw, omega)
    
# def velocity(cf, vel):
#     pos = np.array([0.0, 0.0, 1.0])
#     acc = np.array([0.0, 0.0, 0.0])
#     yaw = 0.0
#     omega = np.array([0.0, 0.0, 0.0])
#     cf.cmdFullState(pos, vel, acc, yaw, omega)
    
# def main():
#     swarm = Crazyswarm()
#     timeHelper = swarm.timeHelper

#     for cf in swarm.allcfs.crazyflies:
#         cf.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
#     timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    
#     while True:
#         for cf in swarm.allcfs.crazyflies:
#             velocity(cf, np.array([0.0,0.0,0.0]))
#         timeHelper.sleep(0.1)
#         print('hello world')

# if __name__ == '__main__':
#     main()





import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray
from crazyflie_py import Crazyswarm
from tf2_msgs.msg import TFMessage
from matplotlib import pyplot as plt
import numpy as np

traj = []
veltraj = []

class Target_executor(Node):

    def __init__(self):
        self.swarm = Crazyswarm()
        self.timeHelper = self.swarm.timeHelper
        super().__init__('target_executor')
        # self.timeHelper.sleep(1.0)
        self.swarm.allcfs.crazyflies[0].takeoff(targetHeight=1.0, duration=3.0)
        self.timeHelper.sleep(4.0)
        # self.swarm.allcfs.goTo([0.0, 0.0, 1.0], 0.0, 3.0)
        # self.timeHelper.sleep(4.0)
        
        self.pose_subscriber = self.create_subscription(
            TFMessage,
            'tf',
            self.position_callback,
            10)
        
        
        self.vel = 0.0
        self.time = None

    def position_callback(self, msg):
        positions = [[i.transform.translation.x, i.transform.translation.y, 1.0, i.header.stamp.sec + i.header.stamp.nanosec/1e9] for i in msg.transforms]
        self.time = msg.transforms[0].header.stamp.sec
        for i, cf in enumerate(self.swarm.allcfs.crazyflies):
            traj.append(positions[i])
            self.position(cf, positions[i][:-1])

    def position(self, cf, pos):
        vel = [float((self.time//2)%5-2), 0.0, 0.0]
        acc = [0.0, 0.0, 0.0]
        yaw = 0.0
        omega = [0.0, 0.0, 0.0]
        print(pos)
        cf.cmdFullState(pos, vel, acc, yaw, omega)

def main(args=None):
    # rclpy.init(args=args)
    try:
        target_executor = Target_executor()
        rclpy.spin(target_executor)
        target_executor.destroy_node()
        rclpy.shutdown()
    except:
        plt.plot(np.array(traj).T[3],np.array(traj).T[0])
        plt.plot(np.array(traj).T[3][:-1],np.diff(np.array(traj).T[0])/np.diff(np.array(traj).T[3]))
        #plt.axis('equal')
        plt.show()

    
if __name__ == '__main__':
    main()