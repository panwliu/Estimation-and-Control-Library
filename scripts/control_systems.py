import numpy as np

# ----------------------- Stability Analysis -----------------------
def pole(A: np.ndarray):
    value, vector = np.linalg.eig(A)
    return value

# ----------------------- Matrix Computations -----------------------
def ctrb(A: np.ndarray, B: np.ndarray) -> np.ndarray:       # Controllability matrix
    n = A.shape[0]
    ctrb_matrix = B
    for i in range(1,n):
        ctrb_matrix = np.hstack( [ctrb_matrix, np.dot( np.linalg.matrix_power(A,i), B) ] )
    return ctrb_matrix

def obsv(A: np.ndarray, C: np.ndarray) -> np.ndarray:       # Observability matrix
    n = A.shape[0]
    obsv_matrix = C
    for i in range(1,n):
        obsv_matrix = np.vstack( [obsv_matrix, np.dot( C, np.linalg.matrix_power(A,i) ) ] )
    # obsv_matrix = ctrb(A.T, C.T).T          # Duality of ctrb and obsv
    return obsv_matrix



if __name__ == "__main__":

    m1, m2, g, l = 10, 1, 9.8, 1
    A, B, C = np.zeros((4,4)), np.zeros((4,1)), np.zeros((1,4))
    A[0,1] = 1
    A[1,2] = -m2/m1 * g
    A[2,3] = 1
    A[3,2] = (m1+m2)/(m1*l) * g
    B[1] = 1/m1
    B[3] = -1/(m1*l)
    C[0,3] = 1

    ctrb_matrix = ctrb(A,B)
    obsv_matrix = obsv(A,C)
    pole(A)