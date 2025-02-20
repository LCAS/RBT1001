import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState

class DirectKinematics(Node):

    def __init__(self):
        # Defines the name of the node
        super().__init__('direct_kinematics')

        # Declare a topic this node will listen to; for every new message, 
        # the callback will be called.
        self.t1_pose_sub = self.create_subscription(
            JointState,
            "/joint_states",
            self.joint_states_cb,
            10
        )

        self.joint_states = [
            0., 0., 0., 0., 0., 0.
        ]

        # The timer_callback will be called continuously until the node is destroyed
        timer_period = 2.  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    # this function receives joint state messages from the robot and stores them
    def joint_states_cb(self, msg):
        self.joint_states = msg.position

    # returns a Homogeneous Rotation matrix
    def HR(self, axis, angle):
        if axis == 'x':
            mat = [
                [1, 0, 0, 0],
                [0, np.cos(angle), -np.sin(angle), 0],
                [0, np.sin(angle), np.cos(angle), 0],
                [0, 0, 0, 1]
            ]
        elif axis == 'y':
            ### TODO complete this function

        
        return np.array(mat)

    # returns a Homogeneous Translation matrix
    def HT(self, axis, distance):
        # TODO complete this function

        return np.array(mat)
    
    ##
    #  This function will be called at a frequency of 1/timer_period Hz
    ##
    def timer_callback(self):
        # Here you should implement the direct kinematics of the robot

        q1 = self.joint_states[0]
        q2 = self.joint_states[1]
        q3 = self.joint_states[2]
        q4 = self.joint_states[3]
        q5 = self.joint_states[4]
        q6 = self.joint_states[5]
        
        T = ## TODO: compose the final transformation matrix 

        # printing
        self.get_logger().info('The final transformation matrix is:')
        with np.printoptions(precision=3, suppress=True):
            self.get_logger().info('\n' + str(T))




def main(args=None):
    rclpy.init(args=args)

    node = DirectKinematics()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
