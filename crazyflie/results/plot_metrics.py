from matplotlib import pyplot as plt
import numpy as np


folder_name = '/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/results/2024_4_29_23h44m42s'

amorti = np.loadtxt(folder_name + '/amorti.txt').T

plt.title("Evolution of the non-covered area")
plt.plot(amorti[0], amorti[1], label="Amorti")
plt.xlabel("Time (s)")
plt.ylabel("Area (m^2)")
plt.legend()
plt.savefig(folder_name + "/coverage.pdf")

# First create some toy data:
t_am = amorti[0]
x_am = amorti[2]
y_am = amorti[3]
goalposex_am = amorti[4]
goalposey_am = amorti[5]

f, (ax1, ax2, ax3) = plt.subplots(1, 3, sharey=False)

ax1.plot(t_am, x_am, label = 'Amorti')
ax2.plot(t_am, y_am, label = 'Amorti')
ax3.plot(x_am, y_am, label = 'Amorti')
ax1.plot(t_am, goalposex_am, label = 'goalposex_am')
ax2.plot(t_am, goalposey_am, label = 'goalposey_am')

ax1.set_title('x Position')
ax2.set_title('y Position')
ax3.set_title('xy Position')

ax1.set_xlabel('time [s]')
ax1.set_ylabel('x [m]')

ax2.set_xlabel('time [s]')
ax2.set_ylabel('y [m]')

ax3.set_xlabel('x [m]')
ax3.set_ylabel('y [m]')

ax1.legend()
ax2.legend()
ax3.legend()

plt.savefig(folder_name + "/position.pdf")

# First create some toy data:
dx_am = amorti[6]
dy_am = amorti[7]
Cx_am = amorti[8]*0.5
Cy_am = amorti[9]*0.5


f, (ax1, ax2) = plt.subplots(1, 2, sharey=False)
ax1.plot(t_am, dx_am,'r', label = 'Amorti')
ax1.plot(t_am, Cx_am,'r--', label = 'target_amorti')
ax2.plot(t_am, dy_am,'r', label = 'Amorti')
ax2.plot(t_am, Cy_am,'r--', label = 'target amorti')

ax1.legend()
ax2.legend()

ax1.set_title('x velocity')
ax2.set_title('y velocity')

ax1.set_xlabel('time [s]')
ax1.set_ylabel(r'$v_x$ [m/s]')

ax2.set_xlabel('time [s]')
ax2.set_ylabel(r'$v_y$ [m/s]')

plt.savefig(folder_name + "/speed.pdf")

ddx_am = amorti[10]
ddy_am = amorti[11]



f, (ax1, ax2) = plt.subplots(1, 2, sharey=False)
N = 20
ax1.plot(t_am, ddx_am,'r', label = 'Amorti')
ax2.plot(t_am, ddy_am,'r', label = 'Amorti')

ax1.legend()
ax2.legend()

ax1.set_title('x acceleration')
ax2.set_title('y acceleration')

ax1.set_xlabel('time [s]')
ax1.set_ylabel(r'$a_x$ [m/$s^2$]')

ax2.set_xlabel('time [s]')
ax2.set_ylabel(r'$a_y$ [m/$s^2$]')

plt.savefig(folder_name + "/acc.pdf")
plt.show()