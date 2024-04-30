import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit

def func(t, A, tau):
    return A*(1 - np.exp(-(t-1)/tau))*0.5

amorti = np.loadtxt('/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/results/2024_4_27_15h17m52s/amorti.txt').T

t = amorti[0]
vx = amorti[6]*0.72
ax = np.diff(vx)/np.diff(t)
N = 10
ax = np.convolve(ax, np.ones(N)/N, mode='same')

data = np.array([t[:-1], ax]).T
data1 = np.array([i for i in filter(lambda x: x[0] < 3.0, data)])
data = np.array([i for i in filter(lambda x: x[0] > 1.0, data1)])
data = np.array(data).T

results = curve_fit(func, data[0], data[1])
plt.plot(data1.T[0], data1.T[1])
plt.plot(data1.T[0], 0.5*(data1.T[0]>1.0), 'r--')
plt.plot(data[0], func(data[0], results[0][0],results[0][1]), 'r')
A = np.round(results[0][0],3)
tau = np.round(results[0][1],3)
plt.text(1.3,0.25,r'$f = 0.5A(1 - e^{-\frac{(t-1)}{\tau}})$', color = 'r', fontsize = 15)
plt.text(1.5,0.15,r'$A ='+ str(A) + '$\n'+r'$\tau ='+ str(tau) + '$', color = 'r', fontsize = 15)

plt.show()
