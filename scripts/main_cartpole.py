import numpy as np
import time
from cartpole_env import CartpoleEnv

env = CartpoleEnv(port_self=18060, port_remote=18080)
print("Environment done")

for k_ep in range(5000):
    time.sleep(0.001)
    action = 0.1 if k_ep%500<250 else -0.1
    env.step(action)
    if k_ep%100 == 99:
        print(env.state_current_)

input("Input Enter to exit")