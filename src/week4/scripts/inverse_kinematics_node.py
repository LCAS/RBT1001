import rclpy
from rclpy.node import Node
from sympy import *
import numpy as np

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from scripts.forward_kinematics import *

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
        
        # print(t)
        p = Matrix([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z
        ])
        R = Quaternion(
            t.transform.rotation.w,
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z
        ).to_rotation_matrix()
        pprint(p)
        pprint(R)
        self.compute_IK(R, p)

    def compute_IK(self, R, p):
        # TODO find the location of the wrist centre
        w_centre = p - .08 * R[:, 0]

        # TODO find q1
        if abs(p[0] - p[1]) < 1e-2 and abs(p[0]) < 1e-2:
            # in this case q1 can take any value
            q1 = (np.random.random() * 2 * np.pi) - np.pi
            print("q1 can be any number due to singularity. Choosing {}.".format(q1))
        else:
            q1 = float(atan2(p[1], p[0]))
            q1_ = float(atan2(-p[1], -p[0]))
            print("q1 can be {} or {}. Choosing {}.".format(q1, q1_, q1))
        
        # TODO find q3
        sinq3 = (p[0]**2 + p[1]**2 + p[2]**2 - 0.05**2 - 0.457**2 - 0.035**2 - 0.33**2 - 0.335**2) / (2 * 0.33 * 0.335) 
        print("sinq3: {}".format(sinq3))
        # re takes the real part of complex numbers
        print(simplify(sqrt(1 - sinq3**2)))
        cosq3 = simplify(sqrt(1 - sinq3**2))
        q3 = N(atan2(sinq3, cosq3))
        q3_ = N(atan2(sinq3, -cosq3))
        # q3 = N(arg(sinq3/cosq3))
        # q3_ = N(arg(-sinq3/cosq3))
        print("q3 can be {} or {}. Choosing {}.".format(q3, q3_, q3))
        

        pprint(w_centre)

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