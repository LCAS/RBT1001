
# importing our custom made modules
import trap_trajectory as trap
import viz_trajectory as viz

import numpy as np
import time
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
    0.6496184220867192,    #arm_1_joint
    -1.1910595388819816,  #arm_2_joint
    -3.141592653589793,  #arm_3_joint
    1.7904987228240348,   #arm_4_joint
    -0.,  #arm_5_joint
    -0.,  #arm_6_joint
    -0.   #arm_7_joint
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
        
        # initialise visualisation module
        viz.init(self)

        period = 0.5
        self.i = 0
        self.timer = self.create_timer(period, self.timer_cb)


    def timer_cb(self):
        # wait until you have the current joint configuration
        while self.last_state is None:
            return 

        if self.i == 0:
            trajectory, times = self.compute_joint_trajectory()
            traj_msg = self.to_JointTrajectory(trajectory, times)
            viz.display(self, traj_msg)
            viz.display(self, traj_msg)
            # time.sleep(5)
            self.plot(trajectory, times)
            self.send_commands(traj_msg)
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
        print("Max speed: {}".format(qdmax))

        # find the joint with the largest distance to target
        dists = [q[1] - q[0] for q in zip(initial_position, target_position)]
        abs_dists = [abs(d) for d in dists]
        max_dist_idx = np.argmax(abs_dists) # get the index

        # find the time needed by the joint with max dist
        total_time = abs_dists[max_dist_idx] / qdmax  + 2 # add 1 second as heuristic
        ticks = 50
        times = [float(i+1)*float(total_time)/ticks for i in range(ticks)]

        print("Joint with largest distance: {}".format(max_dist_idx))
        print("abs distances: {}".format(abs_dists))
        
        # compute trapezoidal profile for all joints
        trajectory = {}
        for i, (q0, qf) in enumerate(zip(initial_position, target_position)):

            # here use the lspb function
            ts, q, qd, ok = trap.lspb(total_time, q0, qf, qdmax, ticks)
            self.get_logger().info('{}: {}'.format(i, ok))
            if ok =="error 2":
                new_qdmax = abs(qf - q0) / (total_time - 1) # removing 1 sec as heuristic

                ts, q, qd, ok = trap.lspb(total_time, q0, qf, new_qdmax, ticks)
                self.get_logger().info('{}: {}'.format(i, ok))
                # # hack continuing at max speed
                # new_ticks = int(abs(qf - q0) / ((total_time / ticks) * abs(qdmax))) + 1
                # if new_ticks > 1:
                #     self.get_logger().info('{}_______'.format("Second attempt"))
                #     new_time = total_time / ticks * new_ticks
                #     ts, q, qd, ok = trap.lspb(new_time, q0, qf, qdmax, new_ticks)
                #     self.get_logger().info('{}: {}'.format(i, ok))
                #     # add the missing ticks by keeping the joint still 
                #     for j in range(new_ticks, ticks):
                #         ts.append(times[j])
                #         q.append(q[j-1])
                #         qd.append(0.0)

            if ts is not None:
                trajectory.update({
                    "joint_{}".format(i+1): {
                        "times": ts,
                        "positions": q,
                        "velocities": qd
                    }
                })
            else: #this case is when the joint is already at target position
                trajectory.update({
                    "joint_{}".format(i+1): {
                        "times": times,
                        "positions": [q0 for i in range(ticks)], 
                        "velocities": [0.0 for i in range(ticks)]
                    }
                })

        return trajectory, times

    def plot(self, traj, times):
        trap.plot(
            [times] * 7, 
            [traj[tjoint]["positions"] for tjoint in traj],
            [traj[tjoint]["velocities"] for tjoint in traj],
            ["q{}".format(i) for i in range(7)])


    def to_JointTrajectory(self, trajectory, times):
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
                trajectory["joint_1"]["positions"][i],
                trajectory["joint_2"]["positions"][i],
                trajectory["joint_3"]["positions"][i],
                trajectory["joint_4"]["positions"][i],
                trajectory["joint_5"]["positions"][i],
                trajectory["joint_6"]["positions"][i],
                trajectory["joint_7"]["positions"][i]
            ]
            msg_point.velocities = [
                trajectory["joint_1"]["velocities"][i],
                trajectory["joint_2"]["velocities"][i],
                trajectory["joint_3"]["velocities"][i],
                trajectory["joint_4"]["velocities"][i],
                trajectory["joint_5"]["velocities"][i],
                trajectory["joint_6"]["velocities"][i],
                trajectory["joint_7"]["velocities"][i]
            ]
            msg_point.accelerations = [0.0] * 7
            secs = int(t)
            msg_point.time_from_start.sec = secs
            msg_point.time_from_start.nanosec = int((t -secs) * 1e9) 

            msg.points.append(msg_point)
        return msg

    def send_commands(self, joint_traj_msg):

        self.publisher_.publish(joint_traj_msg)
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