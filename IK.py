import numpy as np
import time
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


def geometric_jacobian(q):
    J = np.zeros((6, 6))
    Ts = forward_kinematics_all(q)
    p_end = Ts[-1][:3, 3]
    for i in range(6):
        z_i = Ts[i][:3, 2]
        p_i = Ts[i][:3, 3]
        J[:3, i] = np.cross(z_i, (p_end - p_i))
        J[3:, i] = z_i
    return J

def inverse_kinematics_position(q_init, p_target, max_iterations=1000, tolerance=1e-6, alpha=0.01):
    q = q_init.copy()
    start_time  = time.perf_counter()
    for i in range(max_iterations):
        p_current = forward_kinematics(q)[:3, 3]
        delta_p = p_target - p_current

        if np.linalg.norm(delta_p) < tolerance:
            print("iterations:", i)
            break


        J = geometric_jacobian(q)[:3, :6]
        J_pseudo = np.linalg.pinv(J)

        q_dot = J_pseudo @ delta_p
        q += alpha * q_dot
    end_time = time.perf_counter()
    print("Time taken for IK:", end_time - start_time, "seconds")
    return q






if __name__ == "__main__":
    target_position = np.array([0.5, 0.2, 0.3])
    q = np.deg2rad([
    10, 20, 30, 40, 50, 60
    ])
    ik_solution = inverse_kinematics_position(q, target_position)
    print("\nTarget:")
    print(target_position)

    print("\nSolution:")
    print(forward_kinematics(ik_solution)[:3, 3])

    print("\nFinal error:")
    print(np.linalg.norm(target_position - forward_kinematics(ik_solution)[:3, 3]))