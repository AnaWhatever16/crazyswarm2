import matplotlib.pyplot as plt
import numpy as np

folder_name = '/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/results/2024_4_30_14h30m29s'

amorti = np.loadtxt(folder_name + '/amorti.txt').T
amorti[0] = amorti[0]*1000

plt.plot(amorti[0], amorti[1], label = 'x position')
plt.plot(amorti[0], amorti[2], label = 'y position')
plt.title('Position')
plt.xlabel('time (s)')
plt.ylabel('')
plt.savefig(folder_name + '/pos.pdf')

plt.clf()

plt.plot(amorti[0], amorti[3], label = 'x position')
plt.plot(amorti[0], amorti[4], label = 'y position')
plt.savefig(folder_name + '/vel.pdf')

plt.clf()

plt.plot(amorti[0], amorti[5], label = 'x position')
plt.plot(amorti[0], amorti[6], label = 'y position')
plt.savefig(folder_name + '/acc.pdf')
plt.show()