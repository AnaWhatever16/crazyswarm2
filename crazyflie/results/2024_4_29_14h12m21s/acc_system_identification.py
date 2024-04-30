import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


def simple(t, A, tau):
    return A*(1 - np.exp(-(t-1)/tau))

def func(t, A, tau, d):
    return np.array([x for x in map(lambda x: max(x,0.0),A*(1 - np.exp(-(t-1.0-d)/tau)))])

def adapted(t, A, k, d):
    return A*(1 - k*np.exp(-(t-1)/k)/(k-d) - d*np.exp(-(t-1)/d)/(d-k))

amorti = np.loadtxt('/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/results/2024_4_29_14h12m21s/amorti.txt').T

t = amorti[0]
vx = amorti[6]
ax = np.diff(vx)/np.diff(t)
N = 10
ax = np.convolve(ax, np.ones(N)/N, mode='same')

data = np.array([t[:-1], ax]).T
data1 = np.array([i for i in filter(lambda x: x[0] < 3.0, data)])
data = np.array([i for i in filter(lambda x: x[0] > 1.0, data1)])
data = np.array(data).T


results = curve_fit(func, data1.T[0], data1.T[1])
plt.plot(data1.T[0], data1.T[1], label = r'filtered measure of $a_x$')
plt.plot(data1.T[0], (data1.T[0]>1.0), 'g--', label = 'input signal')
plt.plot(data[0], func(data[0], results[0][0],results[0][1],results[0][2]), 'r', label = r'fitted curve of $a_x$')
plt.plot(data[0], adapted(data[0], results[0][0],results[0][1],results[0][2]), 'r', label = r'fitted curve of $a_x$')
plt.plot(data[0], adapted(data[0], results[0][0],results[0][1],results[0][2]), 'g', label = r'fitted curve of $a_x$')


A = np.round(results[0][0],3)
tau = np.round(results[0][1],3)
d = np.round(results[0][2],3)
plt.text(1.3,0.35,r'$f = A(1 - e^{-\frac{(t-1- d)}{\tau}})$', color = 'r', fontsize = 15)
# plt.text(1.5,0.15,r'$A ='+ str(A) + '$\n'+r'$\tau ='+ str(tau) + '$\nd = '+ str(d), color = 'k', fontsize = 12)
plt.legend(loc='upper left')
plt.xlabel('time (s)')
plt.ylabel(r'$a_x$ (m/s$^2$)')
# plt.show()

kps = np.linspace(0,10,10000)
plt.clf()
roots = np.array([np.roots([tau*d, d+tau, 1, kp*A]) for kp in kps])
plt.plot(np.real(roots.T[0]), np.imag(roots.T[0]))
plt.plot(np.real(roots.T[1]), np.imag(roots.T[1]))
plt.plot(np.real(roots.T[2]), np.imag(roots.T[2]))
plt.plot(kps, np.max(roots, axis=1))
print(kps[np.argmin(np.max(roots, axis=1))])
plt.grid(True)
plt.show()