import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import numpy as np
from pprint import pprint
from scipy.spatial.transform import Rotation
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class IKClient(Node):
    def __init__(self):
        super().__init__('ik_analytic_client')

        self.target_pose = None
        # subscribe to /target_pose topic
        self.create_subscription(PoseStamped, 'target_pose', self.target_pose_callback, 10)

        # publish to /joint_states topic
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 10) 

        self.base_frame = "base"

        # Declare and acquire `target_frame` parameter
        self.from_frame_rel = self.declare_parameter(
          'target_frame', "target").get_parameter_value().string_value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

        self.get_logger().info('IK client ready, waiting for target poses...')

    def target_pose_callback(self, msg):
        self.target_pose = msg

        P = np.array([
            self.target_pose.pose.position.x,
            self.target_pose.pose.position.y,
            self.target_pose.pose.position.z
        ])

        try:
            t = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.from_frame_rel,
                rclpy.time.Time())
            self.get_logger().info(
                f'Transform {self.base_frame} to {self.from_frame_rel} is: {t}')
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.base_frame} to {self.from_frame_rel}: {ex}')
        else:
            print("Transform:")
            pprint(t)
            R = Rotation.from_quat([
                t.transform.rotation.x,
                t.transform.rotation.y,
                t.transform.rotation.z,
                t.transform.rotation.w
            ]).as_matrix()

            print("P:")
            pprint(P)
            print("R:")
            pprint(R)
            self.compute_ik(P, R)


    def compute_ik(self, current_P, current_R):
        if self.target_pose is None:
            return
        
        
        # 1. TODO: find Pw, the wrist position
        #    HINT: look at TF in rviz (visualiser) to check the axis direction of the end effector
        Pw = 
        print("Pw: ", Pw)

        # 2. TODO: find q1, the first joint angle
        q1 = 
        print("Q1: ", q1)

        # 3. find Pw' the wrist position in the xy plane
        #    HINT: this allows us to reduce the problem to a 2R planar robot problem for q2 and q3
        Pw_prime = np.array([
            np.linalg.norm(Pw[0:2]), 
            Pw[2] - 0.136, 
            0])
        print("Pw_prime: ", Pw_prime)

        # 4. TODO: find q3 
        #    HINTs: - Use the cosine rule to find q3 (see lecture slides on IK).
        #           - Here you can find two solutions, one for elbow up and one for elbow down.
        #           - You can choose one at random, or use the one that is closest to the current joint state!
       
        q3 =
        print("Q3: ", q3)

        # 5. TODO: find q2
        q2 = 
        print("Q2: ", q2)

        # 6. The actual orientation of link2 and link3 in our robot is different than the one in the 2R problem analysed
        #    so, we need to adjust q2 and q3.
        #    HINT: look at the robot in rviz to understand the orientation of the links and how they differ from 2R axes
        q2 = -q2 - 3*np.pi/2.
        q3 = -q3 - np.pi/2

        print("Q2 adjusted: ", q2)
        print("Q3 adjusted: ", q3)

        # TODO: fill R0123
        #       HINT: You have the analytical form for R0123 in the github wiki page
        R0123 = Rotation.from_matrix([
            ...
        ]).as_matrix()
        # TODO: compute R456
        #       HINT: - You have R0123
        #             - You have R0123456 (current_R)
        #             - how do you get R456? 
        R456 = 
        
        # TODO: compute q4, q5, q6
        #       HINT: - You have the analytic form of R456 in the github wiki page
        #             - Remember to handle the singularity case
        if ...:
            # non singular case
            q5 = 
            q4 = 
            q6 = 
        else:
            # Singluar case
            q5 = 
            q4 = 
            q6 = 


        print("Q4: ", q4)
        print("Q5: ", q5)
        print("Q6: ", q6)

        # check that none of the angles are NaN
        if np.isnan(q1) or np.isnan(q2) or np.isnan(q3) or np.isnan(q4) or np.isnan(q5) or np.isnan(q6):
            return
        
        # Publish the joint states
        self.send_joint_states(q1, q2, q3, q4, q5, q6)

    # Publish the joint states
    def send_joint_states(self, q1, q2, q3, q4, q5, q6):
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
        msg.position = [q1, q2, q3, q4, q5, q6]
        self.joint_states_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ik_client = IKClient()
    
    try:
        rclpy.spin(ik_client)
    except KeyboardInterrupt:
        pass
    
    ik_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()