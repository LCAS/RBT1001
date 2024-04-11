
import trap_trajectory as trap
import numpy as np

import rclpy
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

# max speeds for each joint
max_speed = np.array([
    18.0,
    18.0,
    22.0,
    22.0,
    17.0,
    17.0,
    17.0
])
# rpm to rad/s
max_speed = max_speed / 60.0 * 2 * np.pi

# The position our arm should reach
target_position = [
    1.6,    #arm_1_joint
    -0.93,  #arm_2_joint
    -3.13,  #arm_3_joint
    1.83,   #arm_4_joint
    -1.57,  #arm_5_joint
    -0.62,  #arm_6_joint
    -1.57   #arm_7_joint
]

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('joint_commander')

        # publisher for joint command
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.last_state = None
        self.joint_idx = {}

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
            trajectory, times = self.compute_joint_trajectory()
            self.send_commands(trajectory, times)
            self.i += 1

    def compute_joint_trajectory(self):

        # get initial position
        initial_state = self.last_state
        initial_position = [
            initial_state.position[self.joint_idx["arm_1_joint"]],
            initial_state.position[self.joint_idx["arm_2_joint"]],
            initial_state.position[self.joint_idx["arm_3_joint"]],
            initial_state.position[self.joint_idx["arm_4_joint"]],
            initial_state.position[self.joint_idx["arm_5_joint"]],
            initial_state.position[self.joint_idx["arm_6_joint"]],
            initial_state.position[self.joint_idx["arm_7_joint"]]
        ]
        
        # assume that all the joints move with speed equal to the lowest maximum speed of all joints
        qdmax = max_speed[-1]

        # find the joint with the largest distance to target
        dists = [q[1] - q[0] for q in zip(initial_position, target_position)]
        abs_dists = [abs(d) for d in dists]
        max_dist_idx = np.argmax(abs_dists) # get the index

        # find the time needed by the joint with max dist
        total_time = abs_dists[max_dist_idx] / qdmax
        ticks = 10
        times = [float(i+1)*float(total_time)/ticks for i in range(ticks)]
        
        # compute trapezoidal profile for all joints
        trajectory = []
        for i, (q0, qf) in enumerate(zip(initial_position, target_position)):

            # here use the lspb function
            ts, q, qd, ok = trap.lspb(total_time, q0, qf, qdmax, ticks)
            self.get_logger().info('{}: {}'.format(i, ok))
            if ok =="error 2":
                new_ticks = int(abs(qf - q0) / ((total_time / ticks) * abs(qdmax))) + 1
                if new_ticks > 1:
                    self.get_logger().info('{}_______'.format("Second attempt"))
                    new_time = total_time / ticks * new_ticks
                    ts, q, qd, ok = trap.lspb(new_time, q0, qf, qdmax, new_ticks)
                    self.get_logger().info('{}: {}'.format(i, ok))
                    # add the missing ticks by keeping the joint still 
                    for j in range(new_ticks, ticks):
                        ts.append(times[j])
                        q.append(q[j-1])
                        qd.append(0.0)

            if ts is not None:
                trajectory.append(
                    (ts, q, qd)
                )
            else: #this case is when the joint is already at target position
                trajectory.append((
                    times,
                    [q0 for i in range(ticks)], 
                    [0 for i in range(ticks)]
                ))

        return trajectory, times

    def send_commands(self, trajectory, times):
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
        msg.points = []

        for i, t in enumerate(times):
            # define our  point message
            msg_point = JointTrajectoryPoint()
            msg_point.positions = [
                trajectory[0][i][1],
                trajectory[1][i][1],
                trajectory[2][i][1],
                trajectory[3][i][1],
                trajectory[4][i][1],
                trajectory[5][i][1],
                trajectory[6][i][1]
            ]
            msg_point.velocities = [
                trajectory[0][i][2],
                trajectory[1][i][2],
                trajectory[2][i][2],
                trajectory[3][i][2],
                trajectory[4][i][2],
                trajectory[5][i][2],
                trajectory[6][i][2]
            ]
            msg_point.accelerations = [0.0] * 7
            secs = int(t)
            msg_point.time_from_start.sec = secs
            msg_point.time_from_start.nanosec = int((t -secs) * 1e9) 

            msg.points.append(msg_point)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing commands')


    # joints state callback
    def js_cb(self, msg):

        if len(self.joint_idx) == 0:
            self.joint_idx = dict(zip(
                msg.name,
                [i for i in range(len(msg.name))]
            ))

        self.last_state = msg
        

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