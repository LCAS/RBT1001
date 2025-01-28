import rclpy
from rclpy.node import Node
import random

from sensor_msgs.msg import JointState

class RandomArmController(Node):

    def __init__(self):
        # Defines the name of the node
        super().__init__('random_arm_controller')

        # Declare a new topic this node will publish to
        self.joints_cmd_pub =  ## TODO: Create a publisher for the joint_states topic

        # The timer_callback will be called continuously until the node is destroyed
        timer_period = 2.  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    ##
    #  This function will be called at a frequency of 1/time_period Hz
    #  Here you should put the code that needs to be executed continuously,
    #  regardless of any topic callback, i.e., the logic of your controller.
    ##
    def timer_callback(self):
        # Create a random joint state message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = [
            'joint1_to_base',
            'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5',
            ]
        msg.position = ## TODO: set the joint positions to random values

        ## TODO: Publish the message
        
        # writing ROS logs, can be info(), debug(), warning() or error().
        self.get_logger().info('Publishing: "%s"' % msg)


def main(args=None):
    rclpy.init(args=args)

    random_arm_controller = RandomArmController()

    rclpy.spin(random_arm_controller)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    random_arm_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
