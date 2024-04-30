import matplotlib.pyplot as plt
import numpy as np
from scipy.optimize import curve_fit


def simple(t, A, tau):
    return A*(1 - np.exp(-(t-1)/tau))

def func(t, A, tau, d):
    return np.array([x for x in map(lambda x: max(x,0.0),A*(1 - np.exp(-(t-1.0-d)/tau)))])

def adapted(t, A, k, d):
    return A*(1 - k*np.exp(-(t-1)/k)/(k-d) - d*np.exp(-(t-1)/d)/(d-k))

def adapted2(t, A, k, d):
    return A*(1 - ((d*(d-2*k)*np.cos((t-1)/d) + d*d*np.sin((t-1)/d))*np.exp(-(t-1)/d) + 2*k*k*np.exp(-(t-1)/k))/(d*d-2*d*k+2*k*k))

amorti = np.loadtxt('/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/results/2024_4_29_14h12m21s/amorti.txt').T

t = amorti[0]
vx = amorti[6]
ax = np.diff(vx)/np.diff(t)
N = 2
ax = np.convolve(ax, np.ones(N)/N, mode='same')

data = np.array([t[:-1], ax]).T
data1 = np.array([i for i in filter(lambda x: x[0] < 4.0, data)])
data = np.array([i for i in filter(lambda x: x[0] > 1.0, data1)])
data = np.array([i for i in filter(lambda x: x[0] < 2.0, data)])
data = np.array(data).T

results = curve_fit(func, data1.T[0], data1.T[1])
data1 = np.array([i for i in filter(lambda x: x[0] < 2.0, data1)])

plt.plot(data1.T[0], (data1.T[0]>1.0), 'g--', label = 'input signal')
plt.plot(data1.T[0], data1.T[1], label = r'filtered measure of $a_x$')
plt.plot(data[0], func(data[0], results[0][0],results[0][1],results[0][2]), 'r', label = r'fitted curve of $a_x$')
# plt.plot(data[0], adapted(data[0], results[0][0],results[0][1],results[0][2]), 'g--', label = r'1st order approximation of the fitted curve')
plt.plot(data[0], adapted2(data[0], results[0][0],results[0][1],results[0][2]), 'g', label = r'respons of 3th order system approximation')


A = np.round(results[0][0],3)
tau = np.round(results[0][1],3)
d = np.round(results[0][2],3)
print(A,tau,d)
plt.text(1.3,0.1,r'$f = A(1 - e^{-\frac{(t-1- \tau_d)}{\tau}})$', color = 'r', fontsize = 15)
plt.legend(loc='lower right')
plt.xlabel('time (s)')
plt.ylabel(r'$a_x$ (m/s$^2$)')
plt.title('Curve fitting on an open loop test')
plt.ylim((-0.5,1.1))
plt.savefig('/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/results/2024_4_29_14h12m21s/fit.pdf')

kps = np.linspace(0,10,10000)
plt.clf()
roots = np.array([np.roots([tau*d*d/2,d*d/2+tau*d, d+tau, 1, kp*A]) for kp in kps])

best_index = np.argmin(np.max(roots, axis=1))
best_kp = (kps[best_index])
minimum = np.min(np.max(roots, axis=1))

for i in range(len(roots.T)):
    plt.plot(np.real(roots.T[i]), np.imag(roots.T[i]))
    if(i==0):
        plt.plot(np.real(roots.T[i])[0], np.imag(roots.T[i])[0], 'bo', label = 'roots for k_p = ' + str(kps[0]))
        plt.plot(np.real(roots.T[i])[-1], np.imag(roots.T[i])[-1], 'ro', label = 'roots for k_p = ' + str(kps[-1]))
        plt.plot(np.real(roots.T[i])[best_index], np.imag(roots.T[i])[best_index], 'ko', label = 'roots for k_p = ' + str(np.round(best_kp,2)))
    else:
        plt.plot(np.real(roots.T[i])[0], np.imag(roots.T[i])[0], 'bo')
        plt.plot(np.real(roots.T[i])[-1], np.imag(roots.T[i])[-1], 'ro')
        plt.plot(np.real(roots.T[i])[best_index], np.imag(roots.T[i])[best_index], 'ko')

print(roots[best_index])



plt.grid(True)
plt.legend()
plt.title('Location of the roots of the transfer function')
plt.savefig('/home/matthieu/ros2_ws/src/crazyswarm2/crazyflie/results/2024_4_29_14h12m21s/root_location.pdf')