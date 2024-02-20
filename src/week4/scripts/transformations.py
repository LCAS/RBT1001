# sympy is a library that provide symbolic computation tools
from sympy import *

def HR(axis='x', q='q'):
    """
    Generates a symbolic homogeneous rotation matrix

    Parameters
    ----------
    axis: axis of rotation ('x', 'y' or 'z')
    q: the name of the angle in the symbolic matrix

    Returns
    -------
    (4x4) sympy.Matrix: symbolic homogenous rotation matrix
    """
    if '-' in q:
        _q = q.strip('-')
        qn = symbols(_q)
        qn = "-{}".format(qn)
    else:
        qn = symbols(q)

    R = None
    if axis.lower() =='x':
        # rotation matrix around x axis
        # use the symbol qn to represent the angle variable
        R = Matrix([
            [1, 0, 0, 0],
            [0, cos(qn), -sin(qn), 0],
            [0, sin(qn), cos(qn), 0],
            [0, 0, 0, 1]
        ]) 
    elif axis.lower() =='y':
        # rotation matrix around y axis
        # use the symbol qn to represent the angle variable
        R = Matrix([
            [cos(qn), 0, sin(qn), 0],
            [0, 1, 0, 0],
            [-sin(qn), 0, cos(qn), 0],
            [0, 0, 0, 1]
        ]) 
    elif axis.lower() =='z':
        # rotation matrix around z axis
        # use the symbol qn to represent the angle variable
        R = Matrix([
            [cos(qn), -sin(qn), 0, 0],
            [sin(qn), cos(qn), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]) 
    return R

def HT(x=None, y=None, z=None):
    """
    Generates a symbolic homogeneous translation matrix

    Parameters
    ----------
    x: name of the translation variable on x axis
    y: name of the translation variable on y axis
    z: name of the translation variable on z axis

    Returns
    -------
    (4x4) sympy.Matrix: symbolic homogenous translation matrix
    """
    if x is not None:
        xn = symbols(x)
    else:
        xn = 0

    if y is not None:
        yn = symbols(y)
    else:
        yn = 0

    if z is not None:
        zn = symbols(z)
    else:
        zn = 0

    
    # translation matrix
    # use the symbols xn, yn, zn
    T = Matrix([
        [1, 0, 0, xn],
        [0, 1, 0, yn],
        [0, 0, 1, zn],
        [0, 0, 0, 1]
    ]) 
    return T

if __name__=="__main__":
    v0 = Matrix([1, 2.5, 8, 1])
    T0 = HT(x="x", y="y", z='z').subs({
        'x': -1,
        'y': 2.5,
        'z': -13
    })
    res = T0 * v0
    # pprint(res)
    print("Test 1 passed: {}".format(
        res == Matrix([0, 5, -5, 1])
    ))



    v0 = Matrix([1, 2.5, 8, 1])
    Ry = HR(axis='y', q='q').subs({
        'q': pi
    })
    Rx = HR(axis='x', q='q').subs({
        'q': -pi/2
    })
    # N performs numerical evaluation, evaluating trigon. functions
    res = N(Ry*Rx*v0)
    # pprint(res)
    print("Test 2 passed: {}".format(
        res == Matrix([-1, 8, 2.5, 1])
    ))

    res = N(Rx*Ry*T0*v0)
    # pprint(res)
    print("Test 3 passed: {}".format(
        res == Matrix([0, 5, -5, 1])
    ))

    # Rz_q1 = HR(axis='z', q='q1')
    # Tx_l2 = HT(x='l2')
    # Ry_q2 = HR(axis='y', q='q2')
    # Tz_l1 = HT(z='l1')
    # Ry_q3 = HR(axis='y', q='-q3')
    # Tz_l3 = HT(z='l3')
    # Tx_l4 = HT(x='l4')

    # T3 = Tz_l0 * Rz_q1 * Tx_l2 * Ry_q2 * Tz_l1 * Ry_q3 * Tz_l3 * Tx_l4
    
    # pprint(simplify(T3))

    # pprint(T3.subs({
    #     'q1': 0,
    #     'q2': 0,
    #     'q3': 0
    # }))