import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('joint_commander')

        # publisher for joint command
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/safe_command', 10)

        self.last_state = None
        # subscriber to joint states
        self.js_pub = self.create_subscription(
            JointState,
            "/joint_states",
            self.js_cb,
            10
        )
        
        period = 0.5
        self.i = 0
        self.timer = self.create_timer(period, self.timer_cb)

    def timer_cb(self):
        # wait until you have the current joint configuration
        while self.last_state is None:
            return 

        if self.i == 0:
            self.send_commands()
            self.i += 1

        
    def send_commands(self):
        self.get_logger().info('/joint_states has become available')

        initial_state = self.last_state

        # create joint msg
        msg = JointTrajectory()
        msg.joint_names = [
            "arm_1_joint",
            "arm_2_joint",
            "arm_3_joint",
            "arm_4_joint",
            "arm_5_joint",
            "arm_6_joint",
            "arm_7_joint"
        ]
        # define our target point
        target_point = JointTrajectoryPoint()
        target_point.positions = [
            initial_state.position[0],
            initial_state.position[1],
            initial_state.position[2],
            initial_state.position[3],
            initial_state.position[4],
            initial_state.position[5],
            1.0#initial_state.position[6]
        ]
        target_point.velocities = [0.0] * 7
        target_point.velocities[-1] = 0.5
        target_point.accelerations = [0.0] * 7
        target_point.time_from_start.sec = 3
        # return back to initial 
        final_point = JointTrajectoryPoint()
        final_point.positions = [
            initial_state.position[0],
            initial_state.position[1],
            initial_state.position[2],
            initial_state.position[3],
            initial_state.position[4],
            initial_state.position[5],
            initial_state.position[6]
        ]
        final_point.velocities = [0.0] * 7
        final_point.accelerations = [0.0] * 7
        final_point.time_from_start.sec = 6
        msg.points = [target_point, final_point]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing commands')

    # joints state callback
    def js_cb(self, msg):

        self.last_state = msg
        # self.get_logger().info('Got: {}'.format(msg))


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()