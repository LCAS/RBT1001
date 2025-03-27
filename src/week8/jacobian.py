import numpy as np

# returns a Homogeneous Rotation matrix, angle in radians
def HR(axis, angle):
    if axis == 'x':
        mat = [
            [1, 0, 0, 0],
            [0, np.cos(angle), -np.sin(angle), 0],
            [0, np.sin(angle), np.cos(angle), 0],
            [0, 0, 0, 1]
        ]
    elif axis == 'y':
        mat = [
            [np.cos(angle), 0, np.sin(angle), 0],
            [0, 1, 0, 0],
            [-np.sin(angle), 0, np.cos(angle), 0],
            [0, 0, 0, 1]
        ]
    elif axis == 'z':
        mat = [
            [np.cos(angle), -np.sin(angle), 0, 0],
            [np.sin(angle), np.cos(angle), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]
    return np.array(mat)

# returns a Homogeneous Translation matrix, distance in meters
def HT(axis, distance):
    if axis == 'x':
        mat = [
            [1, 0, 0, distance],
            [0, 1, 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]
    elif axis == 'y':
        mat = [
            [1, 0, 0, 0],
            [0, 1, 0, distance],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]
    elif axis == 'z':
        mat = [
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, distance],
            [0, 0, 0, 1]
        ]
    return np.array(mat)

def dk(q1, q2, q3, q4, q5, q6):
    T = HT('z', 0.136) @ HR('z', q1) @ HR('x', -np.pi/2) @ HR('z', -np.pi/2) @ \
        HR('z', q2) @ HT('x', 0.1) @ HR('z', q3) @ HT('y', 0.107) @ \
        HR('x', -np.pi/2) @ HR('z', q4) @ HR('x', np.pi/2) @ HR('z', q5) @ \
        HR('z', np.pi/2) @ HR('y', np.pi/2) @ HR('z', q6) @ \
        HT('z', 0.065)
    
    return T

def full_jacobian(q1, q2, q3, q4, q5, q6):
    # returns a Homogeneous Rotation matrix, angle in radians
    
    J = np.zeros((6, 6))
    J[:, 0] = J1(q1, q2, q3, q4, q5, q6)
    J[:, 1] = J2(q1, q2, q3, q4, q5, q6)
    J[:, 2] = J3(q1, q2, q3, q4, q5, q6)
    J[:, 3] = J4(q1, q2, q3, q4, q5, q6)
    J[:, 4] = J5(q1, q2, q3, q4, q5, q6)
    J[:, 5] = J6(q1, q2, q3, q4, q5, q6)

    return J


def J1(q1, q2, q3, q4, q5, q6):

    # TODO: implement the Jacobian for the first joint
    jv1 = np.array([0,0,1])
    jw1 = np.array([0,0,1])

    # jw1 is R00 * z0

    # jv1 is R00 * z0 x (P6 - P0)
    P6 = dk(q1, q2, q3, q4, q5, q6)[:3, 3]
    jv1 = np.cross(jw1, P6)

    j1 = np.concatenate((jv1, jw1), axis=0)

    return j1

def J2(q1, q2, q3, q4, q5, q6):

    # TODO: implement the Jacobian for the second joint
    jv2 = np.array([0,0,1])
    jw2 = np.array([0,0,1])

    # jw2 is R01 * z1
    R01 = HR('z', q1) @ HR('x', -np.pi/2) @ HR('z', -np.pi/2) # NOTE: this is taken from the direct kinematics
    R01 = R01[:3, :3] # get only the rotation component
    jw2 = R01 @ jw2

    # This is R01 * z1 x (P6 - P1)
    P6 = dk(q1, q2, q3, q4, q5, q6)[:3, 3]
    P1 = HT('z', 0.136) @ HR('z', q1) @ HR('x', -np.pi/2) @ HR('z', -np.pi/2) # NOTE: this is taken from the direct kinematics
    P1 = P1[:3, 3] # get only the translation component
    jv2 = np.cross(jw2, P6 - P1)


    j2 = np.concatenate((jv2, jw2), axis=0)

    return j2

def J3(q1, q2, q3, q4, q5, q6):

    # TODO: implement the Jacobian for the third joint


    return j3

def J4(q1, q2, q3, q4, q5, q6):

    # TODO: implement the Jacobian for the fourth joint


    return j4

def J5(q1, q2, q3, q4, q5, q6):

    # TODO: implement the Jacobian for the fifth joint

    return j5

def J6(q1, q2, q3, q4, q5, q6):

    # TODO: implement the Jacobian for the sixth joint


    return j6

