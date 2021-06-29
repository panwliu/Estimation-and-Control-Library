import numpy as np
import mujocosim
import os, pathlib, time

def env_by_name(env_name):
    file_path = pathlib.Path( os.path.realpath(__file__) )
    models_path = file_path.parent.parent.__str__() + '/models/'

    if env_name == 'CartPole-v0':
        model_file = 'cartpole.xml'
        env = CartpoleEnv(model_path = models_path + model_file)
    elif env_name == 'CartPole-v1':     # continuous
        model_file = 'cartpole.xml'
        env = CartpoleEnv(model_path = models_path + model_file, discrete=False)
    else:
        print("Can't find env " + env_name)
    
    return env


class CartpoleEnv:
    def __init__(self, model_path="../models/cartpole.xml", discrete=True):
        self.sim = mujocosim.MujocoSim(model_path)
        self.sim_timestep, self.simrate = 0.005, 4
        self.discrete = discrete
        if discrete:
            self.obs_dim, self.act_dim = 4, 2
        else:
            self.obs_dim, self.act_dim = 4, 1

    def step(self, action):
        if self.discrete:
            if action == 0:
                action = -1.0
            elif action == 1:
                action = 1.0
            else:
                print("Unknown action!")
        else:
            action = np.clip(action, -1.0, 1.0)

        for _ in range(self.simrate):
            state = self.sim.step(action)

        # x, theta, x_dot, theta_dot = state
        # r1 = (0.7 - np.abs(x))/0.7 - 0.5
        # r2 = (0.7 - np.abs(theta))/0.7 - 0.5
        # reward = r1 + r2
        reward  = 1

        if np.abs(state[0])>0.7 or np.abs(state[1])>0.7:
            done = True
            # reward = -10
        else:
            done = False

        return state, reward, done

    def reset(self):
        pose = np.random.randn(2)*0.1
        pose = np.clip(pose, -0.2, 0.2)
        pose[0]=0

        state = self.sim.reset(pose)        # state = sim.reset([])
        return state

    def render(self):
        if not hasattr(self, "viewer"):
            self.viewer = mujocosim.MujocoVis(self.sim)
        self.viewer.render()
        



if __name__=="__main__":
    env = env_by_name(env_name="CartPole-v1")

    for _ in range(10):
        done = False

        s = env.reset()
        while not done:
            env.render()

            a = np.zeros(env.act_dim)

            s_, r, done = env.step(a)

            s = s_

