import numpy as np
import time
from cartpole_env import CartpoleEnv
from estimators import UnscentedKalmanFilter
from dynamics import CartpoleDynamics
import matplotlib.pyplot as plt

env = CartpoleEnv(port_self=18060, port_remote=18080)
print("Environment done")

dynamics = CartpoleDynamics(m_c = 1, m_p=0.5, l=1)
estimator = UnscentedKalmanFilter(dynamics=dynamics, x0=np.zeros((4,1)))

n_step = 2000
time = np.zeros(n_step)
states_real = np.zeros((4,n_step))
states_estimated = np.zeros((4,n_step))
for k_step in range(n_step):
    action = 0.1 if k_step%500<250 else -0.1
    state_real = env.step(action)
    state_estimated = estimator.estimate(u=action, y=0)
    
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