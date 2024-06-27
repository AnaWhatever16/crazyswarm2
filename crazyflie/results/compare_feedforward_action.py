from matplotlib import pyplot as plt
import numpy as np


folder_name1 = '/coverage_crazyflie_ws/src/crazyswarm2/crazyflie/results/2024_5_2_16h6m0s'
folder_name2 = '/coverage_crazyflie_ws/src/crazyswarm2/crazyflie/results/2024_5_2_16h9m39s'

normal = np.loadtxt(folder_name1 + '/amorti.txt').T
feedfwd = np.loadtxt(folder_name2 + '/amorti.txt').T


t_am = normal[0]


# First create some toy data:
dx_am = normal[6]
dy_am = normal[7]



plt.plot(t_am, dx_am,'r', label = 'Without feedforward term')


t_am = feedfwd[0]


# First create some toy data:
dx_am = feedfwd[6]
dy_am = feedfwd[7]
Cx_am = feedfwd[8]
Cy_am = feedfwd[9]


plt.plot(t_am, dx_am,'g', label = 'With feedforward term')
plt.plot(t_am, Cx_am,'b--', label = 'Reference speed')

plt.legend()

plt.title('Effect of the feedforward term')

plt.xlabel('time [s]')
plt.ylabel(r'$v_x$ [m/s]')
plt.savefig(folder_name2 + "/feedforward_ramp.pdf")
plt.show()
