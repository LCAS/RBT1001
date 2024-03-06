import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/safe_command', 10)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        self.js_pub = self.create_subscription(
            JointState,
            "/joint_states",
            self.js_cb,
            1
        )

    def timer_callback(self):
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
        point = JointTrajectoryPoint()
        point.positions = [
            0.225,
            -1.45,
            -1.3,
            1.9,
            -1.7,
            1.5,
            -1.0
        ]
        point.time_from_start.sec = 1
        msg.points = [point]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d"' % self.i)
        self.i += 1

    def js_cb(self, msg):
        self.get_logger().info('Got: {}'.format(msg))


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