from matplotlib import pyplot as plt
import numpy as np

folder_name = '/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/results/2024_4_29_20h4m34s'

amorti = np.loadtxt(folder_name + '/amorti.txt').T

t = amorti[0]

vx = amorti[6]

omega = 2.975
def func(t):
    return (1 - (1+omega*(t-1))*np.exp(-omega*(t-1))) * (t > 1.0)

plt.plot(t, t>1.0, 'g--', label = r'$v_{x,ref}$')
plt.plot(t,vx, label = r'$v_{x}$')
plt.plot(t,func(t), label = 'predicted response')
plt.ylabel('$v_{x}$ [m/s]')
plt.xlabel('time [s]')
plt.title(r'Closed loop system respons to a step of $v_{x,ref}$')
plt.legend()
plt.savefig(folder_name + '/CL_verif.pdf')