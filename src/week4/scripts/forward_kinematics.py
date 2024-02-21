from sympy import *
from .transformations import *


def symbolic_FK():
    """
    Generates the symbolic homogeneous transformation matrix for the CR7iA robot (6DOF)
    from the robot base to the end effector frame,
    composing homogeneous rotations and translations.

    Name the joint angles q1,q2,q3,q4,q5,q6 and l0,l1,l2,l3,l4,l5 for translations based on
    the robot datasheet.

    Returns
    -------
    (4x4) sympy.Matrix: symbolic homogenous transformation matrix
    """
    # TODO: write the composition of rotations and translations using HT and HR.
    # NOTE: look at the robot's URDF src/week2/description/urdf/6dof.urdf.xacro
    #      0 to 4...
    # NOTE: pay attention to the rotation, you can give neg angles e.g. HR(q="-q3")
    T = HT(z="l0")*HR(axis="z", q="q1")*HT(x="l3")*HR(axis="y", q="q2")*HT(z="l1")*HR(axis="y", q="-q3")*HT(z="l2")*HT(x="l4")
    #      ...wrist and tool
    T = T*HR(axis="x", q="-q4")*HR(axis="y", q="-q5")*HR(axis="x", q="-q6")*HT(x="l5")

    return T

def numeric_FK(T, q=(0,0,0,0,0,0)):
    """
    Substitutes values for the angles in the symbolic transformation.

    Parameters
    ----------
    T: The symbolic transformation matrix
    q: the values of the joint angles

    Returns
    -------
    (4x4) sympy.Matrix: symbolic homogenous transformation matrix
    """
    T = T.subs({
        'q1': q[0],
        'q2': q[1],
        'q3': q[2],
        'q4': q[3],
        'q5': q[4],
        'q6': q[5],
        'l0': 0.457,
        'l1': 0.330,
        'l2': 0.035,
        'l3': 0.05,
        'l4': 0.335,
        'l5': 0.16
    })

    return T


if __name__ == "__main__":

    symT = symbolic_FK()

    print("Symbolic T")
    # pprint stands for pretty-print, and shows matrices in a human readable way
    # simplify reduces expressions to the simplest form
    pprint(simplify(symT))

    numT = numeric_FK(symT)

    print("Numeric T")
    # N performs numerical evaluation, evaluating trigon. functions
    pprint(N(numT))

