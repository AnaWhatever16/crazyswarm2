"""Takeoff-hover-land for one CF. Useful to validate hardware config."""

from crazyflie_py import Crazyswarm


TAKEOFF_DURATION = 4.0
HOVER_DURATION = 1.0


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    swarm.allcfs.takeoff(targetHeight=1.0, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)



if __name__ == '__main__':
    main()

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
    
# def main():
#     swarm = Crazyswarm()
#     timeHelper = swarm.timeHelper

#     for cf in swarm.allcfs.crazyflies:
#         cf.takeoff(targetHeight=0.5, duration=TAKEOFF_DURATION)
#     timeHelper.sleep(TAKEOFF_DURATION + HOVER_DURATION)
    
#     for i in range(200):
#         for cf in swarm.allcfs.crazyflies:
#             position(cf, np.array([cos(i/10)-1,sin(i/10),1.0])+cf.initialPosition)
#         timeHelper.sleep(0.1)
#         print('hello world')

# if __name__ == '__main__':
#     main()
