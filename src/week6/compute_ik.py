import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.cli = self.create_client(GetPositionIK, 'compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPositionIK.Request()

    def send_request(self):
        self.req.ik_request.group_name = 'mecharm'
        self.req.ik_request.robot_state.joint_state.name = [
            'joint1_to_base',
             'joint2_to_joint1',
            'joint3_to_joint2',
            'joint4_to_joint3',
            'joint5_to_joint4',
            'joint6_to_joint5'
            ]
        self.req.ik_request.robot_state.joint_state.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Define the target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = 'base'
        target_pose.pose.position.x = 0.2
        target_pose.pose.position.y = 0.0
        target_pose.pose.position.z = 0.243
        #[0.500, -0.500, 0.500, -0.500]
        target_pose.pose.orientation.x = 0.50
        target_pose.pose.orientation.y = -0.50
        target_pose.pose.orientation.z = 0.50
        target_pose.pose.orientation.w = -0.50
        self.req.ik_request.pose_stamped = target_pose

        self.future = self.cli.call_async(self.req)

def main(args=None):
    rclpy.init(args=args)
    ik_client = IKClient()
    ik_client.send_request()

    while rclpy.ok():
        rclpy.spin_once(ik_client)
        if ik_client.future.done():
            try:
                response = ik_client.future.result()
            except Exception as e:
                ik_client.get_logger().info('Service call failed %r' % (e,))
            else:
                ik_client.get_logger().info('IK solution: %s' % str(response.solution.joint_state.position))
            break

    ik_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()