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
        p1 = JointTrajectoryPoint()
        p1.positions = [
            0.09,
            -0.68,
            -3.11,
            2.09,
            -1.12,
            -0.03,
            -2.0
        ]
        p1.velocities = [0.5] * 7
        p1.accelerations = [0.0] * 7
        p1.time_from_start.sec = 3
        p2 = JointTrajectoryPoint()
        p2.positions = [
            0.09,
            -0.73,
            -2.94,
            1.83,
            -1.12,
            -0.03,
            -2.0
        ]
        p2.velocities = [0.0] * 7
        p2.velocities[1] = 0.5
        p2.velocities[2] = 0.5
        p2.velocities[3] = 0.5
        p2.accelerations = [0.0] * 7
        p2.time_from_start.sec = 6
        p3 = JointTrajectoryPoint()
        p3.positions = [
            0.09,
            -0.72,
            -2.94,
            2.18,
            -1.12,
            -0.03,
            -2.04
        ]
        p3.velocities = [0.0] * 7
        p3.velocities[1] = 0.5
        p3.velocities[3] = 0.5
        p3.velocities[-1] = 0.5
        p3.accelerations = [0.0] * 7
        p3.time_from_start.sec = 9
        p4 = JointTrajectoryPoint()
        p4.positions = [
            0.09,
            -0.73,
            -2.94,
            1.83,
            -1.12,
            -0.03,
            -2.0
        ]
        p4.velocities = [0.0] * 7
        p4.velocities[1] = 0.5
        p4.velocities[3] = 0.5
        p4.velocities[-1] = 0.5
        p4.accelerations = [0.0] * 7
        p4.time_from_start.sec = 12
        msg.points = [p1,p2,p3,p4]
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
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()