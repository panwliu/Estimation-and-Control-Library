import numpy as np

def hat(a:np.mat):
    a = np.array(a).reshape(1,-1)[0,:]
    a_hat = np.mat([ [0,-a[2],a[1]], [a[2],0,-a[0]], [-a[1],a[0],0] ])
    return a_hat

def dehat(A:np.mat):
    a = np.mat([[A[2,1]], [A[0,2]], [A[1,0]]])
    return a

def quat2rotm(quat: np.mat):
    q = np.array(quat).reshape(1,-1)[0,:]
    q = q / np.linalg.norm(q)
    
    theta = 2 * np.arccos(q[0])
    omega = [0,0,0] if theta==0 else q[1:]/np.sin(theta/2)      # ||omega|| = 1

    I = np.mat("1,0,0; 0,1,0; 0,0,1")
    omega_hat = hat(omega)

    R = I + omega_hat*np.sin(theta) + omega_hat*omega_hat*(1-np.cos(theta))     # Rodrigue's formula

    return R

def eul2rotm(eul_angles:np.mat, sequence:str="ZYX"):
    angles = np.array(eul_angles).reshape(1,-1)[0,:]
    n_rot = angles.size
    R_all = []
    for i in range(n_rot):
        angle = angles[i]
        axis = sequence[i]
        s = np.sin(angle)
        c = np.cos(angle)
        if axis=='x' or axis=='X':
            x = np.mat("1;0;0")
            y = np.mat([ [0], [c], [s] ])
            z = np.mat([ [0], [-s], [c] ])
        elif axis=='y' or axis=='Y':
            x = np.mat([ [c], [0], [-s] ])
            y = np.mat("0;1;0")
            z = np.mat([ [s], [0], [c] ])
        elif axis=='z' or axis=='Z':
            x = np.mat([ [c], [s], [0] ])
            y = np.mat([ [-s], [c], [0] ])
            z = np.mat("0;0;1")
        else:
            print("Wrong aixs")
        R = np.hstack( [x,y,z] )
        R_all.append(R)
    
    res = np.mat("1,0,0; 0,1,0; 0,0,1")
    for i in reversed(range(n_rot)):
        res = R_all[i]*res

    return res



