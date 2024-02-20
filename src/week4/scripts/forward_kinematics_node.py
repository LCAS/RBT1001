
import rclpy
from rclpy.node import Node
from .forward_kinematics import *
from sympy import *

class ForwardKinematics(Node):
    def __init__(self):
        # Defines the name of the node
        super().__init__('forward_kinematics')

        # TODO subscribe to the topic /join_states to read the joint 
        #      angle values in real time. Use the callback joint_angles_cb

        # TODO compute the symbolic transformation matrix
        self.symT = symbolic_FK()

    def joint_angles_cb(self, msg):
        # get the angles values
        q = msg.position

        # TODO insert the values to compute the numerical tranformation matrix
        #      and print it 
        numT = numeric_FK(self.symT, q=q)

        pprint(numT)


def main(args=None):
    rclpy.init(args=args)

    fk = ForwardKinematics()

    rclpy.spin(fk)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fk.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()