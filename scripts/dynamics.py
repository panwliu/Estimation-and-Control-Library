import numpy as np

class BaseDynamics:
    def next_state(self, x0, u, dt):
        pass

class CartpoleDynamics(BaseDynamics):
    def __init__(self, m_c, m_p, l):
        self.m1, self.m2, self.l, self.g = m_c, m_p, l, 9.8

    def dynamic_model(self, x: np.ndarray, u: np.ndarray) -> np.ndarray:
        m1, m2, l, g = self.m1, self.m2, self.l, 9.8

        px, theta, vx, theta_d = x

        H_inv = np.array([ [1/(m1+m2-m2*np.cos(theta)**2), -m2*np.cos(theta)/(m1+m2-m2*np.cos(theta)**2)],
            [-np.cos(theta)/(m1*l+m2*l-m2*l*np.cos(theta)**2), (m1+m2)/(m1*l+m2*l-m2*l*np.cos(theta)**2)] ])
        C = np.array([ [0, -m2*l*theta_d*np.sin(theta)], [0, 0] ])
        G = np.array([ [0], [-g*np.sin(theta)] ])
        B = np.array([ [1], [0] ])

        x_dot = np.zeros(shape=(4,1))
        x_dot[0:2] = x[2:4]
        x_dot[2:4] = np.dot( H_inv, -np.dot(C, x[2:4])-G+np.dot(B,u) )

        return x_dot

    def next_state(self, x0, u, dt):
        return x0 + self.dynamic_model(x0,u)*dt

    def rk4(self, ode, x0: np.ndarray, dt, n_step, u) -> np.ndarray:
        n_var = np.size(x0)
        x = np.zeros((n_var,n_step+1))
        x[:, 0] = x0[:, 0]
        h = dt/n_step

        for k_step in range(n_step):
            xk = x[:,k_step].reshape(-1,1)

            k1 = h*ode(xk, u)
            k2 = h*ode(xk+k1/2, u)
            k3 = h*ode(xk+k2/2, u)
            k4 = h*ode(xk+k3, u)

            x[:,k_step+1] = (xk + (k1 + 2*k2 + 2*k3 + k4)/6)[:,0]

        return x[:,-1].reshape(-1,1)
