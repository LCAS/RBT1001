import math

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class InverseKinematics(Node):
    def __init__(self):
        super().__init__('inverse_kinematics_node')


        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)


    def get_transform(self, base_frame, target_frame):

        # Declare and acquire `target_frame` parameter
        from_frame_rel = self.declare_parameter(
          'target_frame', target_frame).get_parameter_value().string_value

        # Look up for the transformation between target_frame and turtle2 frames
        # and send velocity commands for turtle2 to reach target_frame
        while True:
            try:
                t = self.tf_buffer.lookup_transform(
                    base_frame,
                    from_frame_rel,
                    rclpy.time.Time())
                self.get_logger().info(
                    f'Transform {base_frame} to {from_frame_rel} is: {t}')
                return t
            except TransformException as ex:
                self.get_logger().info(
                    f'Could not transform {base_frame} to {from_frame_rel}: {ex}')


def main():
    rclpy.init()
    node = InverseKinematics()


    node.get_transform("arm_1_link", "arm_7_link")


    # while True:
    #     try:
    #         rclpy.spin_once(node)
    #     except KeyboardInterrupt:
    #         break

    rclpy.shutdown()

if __name__== "__main__":
    main()