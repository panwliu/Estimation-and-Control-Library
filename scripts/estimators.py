import numpy as np
from dynamics import BaseDynamics

class UnscentedKalmanFilter:
    def __init__(self, dynamics: BaseDynamics, x0: np.ndarray):
        self.dynamics = dynamics
        self.n_dim = x0.shape[0]

        self.x_minus = x0
        self.x_plus = x0
        self.P_minus = np.zeros((self.n_dim, self.n_dim))
        self.P_plus = np.zeros_like(self.P_minus)
        

    def estimate(self, u, y):
        self.predict(u)
        self.update()

        return self.x_plus[:,-1].reshape((-1,1))
    
    def predict(self, u):
        x0 = self.x_plus
        dt = 1e-3

        self.x_minus = self.dynamics.next_state(x0, u, dt)

    def update(self):
        self.x_plus = self.x_minus