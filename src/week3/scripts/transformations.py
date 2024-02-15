from sympy import *
import math

def HR(axis='x', q='q'):
    if '-' in q:
        _q = q.strip('-')
        qn = symbols(_q)
        qn = "-{}".format(qn)
    else:
        qn = symbols(q)

    R = None
    if axis.lower() =='x':
        R = Matrix([
            [1, 0, 0, 0],
            [0, cos(qn), -sin(qn), 0],
            [0, sin(qn), cos(qn), 0],
            [0, 0, 0, 1]
        ]) 
    elif axis.lower() =='y':
        R = Matrix([
            [cos(qn), 0, sin(qn), 0],
            [0, 1, 0, 0],
            [-sin(qn), 0, cos(qn), 0],
            [0, 0, 0, 1]
        ]) 
    elif axis.lower() =='z':
        R = Matrix([
            [cos(qn), -sin(qn), 0, 0],
            [sin(qn), cos(qn), 0, 0],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ]) 
    return R

def HT(x=None, y=None, z=None):
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

    T = Matrix([
        [1, 0, 0, xn],
        [0, 1, 0, yn],
        [0, 0, 1, zn],
        [0, 0, 0, 1]
    ]) 
    return T

if __name__=="__main__":

    Tz_l0 = HT(z='l0')
    Rz_q1 = HR(axis='z', q='q1')
    Tx_l2 = HT(x='l2')
    Ry_q2 = HR(axis='y', q='q2')
    Tz_l1 = HT(z='l1')
    Ry_q3 = HR(axis='y', q='-q3')
    Tz_l3 = HT(z='l3')
    Tx_l4 = HT(x='l4')


    T3 = Tz_l0 * Rz_q1 * Tx_l2 * Ry_q2 * Tz_l1 * Ry_q3 * Tz_l3 * Tx_l4
    
    pprint(simplify(T3))

    # pprint(T3.subs({
    #     'q1': 0,
    #     'q2': 0,
    #     'q3': 0
    # }))