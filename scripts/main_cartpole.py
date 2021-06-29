import numpy as np
import time
from envs import env_by_name
from dynamics import CartpoleDynamics
from estimators import UnscentedKalmanFilter
import matplotlib.pyplot as plt

env = env_by_name(env_name="CartPole-v1")
print("Environment done")

wn_sigma = np.array([1, 1, 2, 2])*1e-3
dynamics = CartpoleDynamics(m_c = 1, m_p=0.5, l=1, wn_sigma=wn_sigma)
H = np.array([1,0,0,0]).reshape((1,-1))     # measuring position makes the system observable
Q = np.diag(wn_sigma**2)
vn_sigma = 1e-2
R = vn_sigma**2
estimator = UnscentedKalmanFilter(dynamics=dynamics, x0=np.zeros((4,1)), H=H, Q=Q, R=R)

n_step = 1500
time = np.zeros(n_step)
states_real = np.zeros((4,n_step))
states_estimated = np.zeros((4,n_step))
for k_step in range(n_step):
    action = 0.1 if k_step%500<250 else -0.1
    state_real, _, _ = env.step(action)
    y = state_real[3] + vn_sigma*np.random.randn(1)[0]
    state_estimated = estimator.estimate(u=action, y=y)
    
    time[k_step] = state_real[2]
    states_real[:,k_step] = state_real[3:]
    states_estimated[:,k_step] = state_estimated[:,0]

plt.figure()
plt.subplot(2,2,1)
l1, = plt.plot(time[::20], states_real[0, ::20])
l2, = plt.plot(time[::20], states_estimated[0, ::20])
plt.legend([l1, l2], ["real","estimate"], loc="upper right")
plt.xlabel("Time (s)")
plt.ylabel("Position (m)")
plt.title("Position")

plt.subplot(2,2,2)
l1, = plt.plot(time[::20], states_real[1, ::20]*180/np.pi)
l2, = plt.plot(time[::20], states_estimated[1, ::20]*180/np.pi)
plt.legend([l1, l2], ["real","estimate"], loc="upper right")
plt.xlabel("Time (s)")
plt.ylabel("Angle (degree)")
plt.title("Angle")

plt.subplot(2,2,3)
l1, = plt.plot(time[::20], states_real[2, ::20])
l2, = plt.plot(time[::20], states_estimated[2, ::20])
plt.legend([l1, l2], ["real","estimate"], loc="upper right")
plt.xlabel("Time (s)")
plt.ylabel("Velocity (m/s)")
plt.title("Velocity")

plt.subplot(2,2,4)
l1, = plt.plot(time[::20], states_real[3, ::20]*180/np.pi)
l2, = plt.plot(time[::20], states_estimated[3, ::20]*180/np.pi)
plt.legend([l1, l2], ["real","estimate"], loc="upper right")
plt.xlabel("Time (s)")
plt.ylabel("Angular velocity (degree/s)")
plt.title("Angular velocity")

plt.show()

input("Input Enter to exit")