from matplotlib import pyplot as plt
import numpy as np

amorti = np.loadtxt("/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/scripts/amorti.txt").T
non_amorti = np.loadtxt("/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/scripts/non_amorti.txt").T

# plt.title("Evolution of the non-covered area")
# plt.plot(amorti[0], amorti[1], label="Amorti")
# plt.plot(non_amorti[0], non_amorti[1], label="Non amorti")
# plt.xlabel("Time (s)")
# plt.ylabel("Area (m^2)")
# plt.legend()
# plt.show()

# First create some toy data:
t_am = amorti[0]
x_am = amorti[2]
y_am = amorti[3]

t_non_am = non_amorti[0]
x_non_am = non_amorti[2]
y_non_am = non_amorti[3]


# f, (ax1, ax2, ax3) = plt.subplots(1, 3, sharey=False)
# ax1.plot(t_am, x_am, label = 'Amorti')
# ax2.plot(t_am, y_am, label = 'Amorti')
# ax3.plot(x_am, y_am, label = 'Amorti')

# ax1.plot(t_non_am, x_non_am, label = 'Non-amorti')
# ax2.plot(t_non_am, y_non_am, label = 'Non-amorti')
# ax3.plot(x_non_am, y_non_am, label = 'Non-amorti')

# ax1.legend()
# ax2.legend()
# ax3.legend()

plt.show()

# First create some toy data:
dx_am = amorti[4]
dy_am = amorti[5]
Cx_am = amorti[6]*0.2
Cy_am = amorti[7]*0.2

dx_non_am = non_amorti[4]
dy_non_am = non_amorti[5]
Cx_non_am = non_amorti[6]*0.2
Cy_non_am = non_amorti[7]*0.2

f, (ax1, ax2) = plt.subplots(1, 2, sharey=False)
ax1.plot(t_am, dx_am,'r', label = 'Amorti')
ax1.plot(t_am, Cx_am,'r--', label = 'target_amorti')
ax2.plot(t_am, dy_am,'r', label = 'Amorti')
ax2.plot(t_am, Cy_am,'r--', label = 'target amorti')

ax1.plot(t_non_am, dx_non_am,'b', label = 'Non-amorti')
ax1.plot(t_non_am, Cx_non_am,'b--', label = 'target non-amorti')
ax2.plot(t_non_am, dy_non_am,'b', label = 'Non-amorti')
ax2.plot(t_non_am, Cy_non_am,'b--', label = 'target non-amorti')

ax1.legend()
ax2.legend()

plt.show()
