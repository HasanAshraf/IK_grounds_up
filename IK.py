import numpy as np

def dh_transform(a, alpha, d, theta):

    ca = np.cos(alpha)
    sa = np.sin(alpha)

    ct = np.cos(theta)
    st = np.sin(theta)

    T = np.array([
        [ct, -st * ca,  st * sa, a * ct],
        [st,  ct * ca, -ct * sa, a * st],
        [0,       sa,       ca,      d],
        [0,        0,        0,      1]
    ])

    return T

DH = np.array([
    [0.00,  np.pi/2, 0.40],
    [0.30,  0.00,    0.00],
    [0.20,  0.00,    0.00],
    [0.00,  np.pi/2, 0.40],
    [0.00, -np.pi/2, 0.00],
    [0.00,  0.00,    0.10]
])

def forward_kinematics(q):
    T = np.eye(4)
    for i in range(6):
        a, alpha, d = DH[i]
        theta = q[i]
        T_i = dh_transform(a, alpha, d, theta)
        T = np.dot(T, T_i)
    return T

q = np.deg2rad([
    10, 20, 30, 40, 50, 60
])

def forward_kinematics_all(q):
    T = np.eye(4)
    Ts = [T.copy()]
    for i in range(6):
        a, alpha, d = DH[i]
        theta = q[i]
        T_i = dh_transform(a, alpha, d, theta)
        T = np.dot(T, T_i)
        Ts.append(T.copy())
    return Ts

q = np.deg2rad([
    10, 20, 30, 40, 50, 60
])

def geometric_jacobian(q):
    
    a
T06 = forward_kinematics_all(q)
print("Transformation Matrix T06:")
print(T06)