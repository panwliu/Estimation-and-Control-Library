import numpy as np
from scipy import linalg
from dynamics import BaseDynamics

class UnscentedKalmanFilter:
    def __init__(self, dynamics: BaseDynamics, x0: np.ndarray, H, Q, R):
        self.dynamics = dynamics
        self.n_dim = x0.shape[0]

        self.x_minus = x0
        self.x_plus = x0
        self.P_minus = np.zeros((self.n_dim, self.n_dim))
        self.P_plus = np.zeros_like(self.P_minus)

        self.H = H
        self.Q = Q
        self.R = R
        

    def estimate(self, u, y):
        self.predict(u)
        self.update(y)

        return self.x_plus
    
    def predict(self, u):
        n_dim = self.n_dim
        x0 = self.x_plus
        P = self.P_plus
        dt = 1e-3

        x_sigpts = np.zeros((n_dim, 2*n_dim))
        nP_sqrt = linalg.sqrtm( n_dim*P )
        for i in range(n_dim):
            x_sigpts[:,2*i] = x0[:,0] + nP_sqrt[i,:]
            x_sigpts[:,2*i+1] = x0[:,0] - nP_sqrt[i,:]

        f_x = np.zeros((n_dim, 2*n_dim))
        for i in range(2*n_dim):
            f_x[:,i] = self.dynamics.next_state(x_sigpts[:,i].reshape((-1,1)), u, dt)[:,0]

        self.x_minus = np.mean(f_x, axis=1).reshape((-1,1))
        self.P_minus = 1/(2*n_dim) * np.dot( (f_x - self.x_minus), (f_x - self.x_minus).T ) + self.Q

    def update(self, y):
        H = self.H
        R = self.R
        P_minus = self.P_minus
        x_minus = self.x_minus

        Kf = np.linalg.multi_dot([P_minus, H.T, np.linalg.inv(np.linalg.multi_dot([H, P_minus, H.T]) + R ) ])
        x_plus = x_minus + np.dot(Kf, y-np.dot(H,x_minus))
        P_plus = np.dot(np.eye(self.n_dim)-np.dot(Kf,H), P_minus)

        self.x_plus = x_plus
        self.P_plus = P_plus