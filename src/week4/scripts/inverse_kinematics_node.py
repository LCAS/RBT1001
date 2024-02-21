import rclpy
from rclpy.node import Node
import time
from sympy import *

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from .forward_kinematics import *

class InverseKinematics(Node):
    def __init__(self):
        # Defines the name of the node
        super().__init__('inverse_kinematics')

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Declare and acquire `target_frame` parameter
        self.declare_parameter(
          'target_frame', 'tool_link')
        self.target_frame = self.get_parameter(
            'target_frame').get_parameter_value().string_value
        
        # Call on_timer function every second
        self.timer = self.create_timer(1.0, self.on_timer)

    def on_timer(self):
        to_frame_rel = "base_link"
        from_frame_rel = self.target_frame
        t = None
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
            return
        
        print(t)
        p = Matrix([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ])
        R = Quaternion(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w
        ).to_rotation_matrix()
        # pprint(p)
        pprint(R)
        self.compute(p)

    def compute(self, p):
        # TODO find the location of the wrist centre
        
        w_centre = p

def main(args=None):
    rclpy.init(args=args)

    ik = InverseKinematics()

    rclpy.spin(ik)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    ik.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()