from matplotlib import pyplot as plt
import numpy as np


folder_name = '/coverage_crazyflie_ws/src/crazyswarm2/crazyflie/results/2024_5_13_15h0m50s'

amorti = np.loadtxt(folder_name + '/amorti.txt').T

t = amorti[0]
F = amorti[1]
plt.plot(t,F)
plt.show()

polygon = np.array([[1,1],[-1,1],[-1,-1],[1,-1],[1,1]])

plt.plot(polygon[:,0],polygon[:,1], 'r', label = "ROI")

theta = np.linspace(0,2*np.pi, 100)

colors = ['b','g','c','m','y','k','orange','purple','brown']

for i in range(1,(len(amorti))//2):
    plt.plot(amorti[2*i],amorti[2*i+1], label = "drone "+ str(i), color = colors[i-1])
    plt.plot(amorti[2*i][-1],amorti[2*i+1][-1],'o', color = colors[i-1])
    plt.plot(amorti[2*i][-1] + np.cos(theta), amorti[2*i+1][-1] + np.sin(theta), color = colors[i-1])
plt.axis('equal')
plt.legend()
plt.show()