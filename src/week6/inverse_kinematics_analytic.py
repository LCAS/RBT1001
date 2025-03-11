import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
from pprint import pprint
from scipy.spatial.transform import Rotation

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_analytic_client')

        self.target_pose = None
        # subscribe to /target_pose topic
        self.create_subscription(PoseStamped, 'target_pose', self.target_pose_callback, 10)
        

        self.get_logger().info('IK client ready, waiting for target poses...')

    def target_pose_callback(self, msg):
        self.target_pose = msg

        P = np.array([
            self.target_pose.pose.position.x,
            self.target_pose.pose.position.y,
            self.target_pose.pose.position.z
        ])
        R = Rotation.from_quat([
            self.target_pose.pose.orientation.x,
            self.target_pose.pose.orientation.y,
            self.target_pose.pose.orientation.z,
            self.target_pose.pose.orientation.w
        ]).as_matrix()

        print("P:")
        pprint(P)
        print("R:")
        pprint(R)

        # Compute the IK solution
        self.compute_ik(P, R)

    def compute_ik(self, current_P, current_R):
        if self.target_pose is None:
            return
        
        # TODO: implement the analytic inverse kinematics solution here
        

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