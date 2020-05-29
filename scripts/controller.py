import numpy as np
from scipy import linalg

# ----------------------- Optimal Control -----------------------
def lqr(A: np.matrix, B: np.matrix, Q: np.matrix, R: np.matrix):
    P = linalg.solve_continuous_are(A, B, Q, R)
    K = R.I * B.T * P
    
    return K

if __name__ == "__main__":
    A = np.mat("0 0 1 0; 0 0 0 1; 0 -4.9 0 0; 0 14.7 0 0")
    B = np.mat("0; 0; 2; -1")
    Q = np.mat("1 0 0 0; 0 1 0 0; 0 0 0.5 0; 0 0 0 0.5")
    R = np.mat("1")

    lqr(A, B, Q, R)