import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped

class IKClient(Node):
    def __init__(self):
        super().__init__('ik_client')
        self.cli = self.create_client(GetPositionIK, '/compute_ik')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = GetPositionIK.Request()
        self.target_pose = None
        # subscribe to /target_pose topic
        self.create_subscription(PoseStamped, 'target_pose', self.target_pose_callback, 10)
        self.future = None
        self.get_logger().info('IK client ready, waiting for target poses...')

    def target_pose_callback(self, msg):
        self.target_pose = msg
        self.send_request()

    def send_request(self):
        if self.target_pose is None:
            return
            
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
        target_pose.pose.position.x = self.target_pose.pose.position.x
        target_pose.pose.position.y = self.target_pose.pose.position.y
        target_pose.pose.position.z = self.target_pose.pose.position.z
        target_pose.pose.orientation.x = self.target_pose.pose.orientation.x
        target_pose.pose.orientation.y = self.target_pose.pose.orientation.y
        target_pose.pose.orientation.z = self.target_pose.pose.orientation.z
        target_pose.pose.orientation.w = self.target_pose.pose.orientation.w
        self.req.ik_request.pose_stamped = target_pose

        self.future = self.cli.call_async(self.req)
        self.future.add_done_callback(self.ik_response_callback)
        
    def ik_response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info('IK solution: %s' % str(response.solution.joint_state.position))
        except Exception as e:
            self.get_logger().error('Service call failed %r' % (e,))

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