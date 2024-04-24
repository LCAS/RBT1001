import numpy as np
import matplotlib.pyplot as plt
import sys
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


# Linear Segments with Parabolic Blends
# Computes the lspb for a given joint given
#  tf:    the final time [secs]
#  q0:    initial joint value [rad]
#  qf:    final joint value [rad]
#  qdmax: maximum joint velocity [rad/s]
#  ticks: number of subdivision points in the trajectory 
# Returns:
#  ts: corresponding list of times
#  qs: list of joint values defined along the trajectory
#  qds: joint velocities
def lspb(tf, q0, qf, qdmax, ticks):
    qdmax = abs(qdmax) * np.sign(qf - q0)
    print("[LSPB] Input data\n tf:{}, q0:{}, qf:{}, qdmax:{}, ticks:{}".format(tf, q0, qf, qdmax, ticks))

    if abs(qdmax) < abs(qf-q0)/tf:
        print("qdmax too small")
        return None, None, None, "error 1"
    elif abs(qdmax) > 2*abs(qf-q0)/tf:
        print("qdmax too large")
        return None, None, None, "error 2"

    # find tb
    # print("[LSPB] \t qdmax:{}\t q0 - qf:{}\t qdmax * tf:{}".format(qdmax, q0 - qf, qdmax * tf))
    tb = (q0 - qf + (qdmax * tf)) / qdmax  # trapezoidal case case
    print("[LSPB] \t tb:{}".format(tb))

    # find max acceleration
    qddmax = qdmax / tb
    print("[LSPB] \t qddmax:{}".format(qddmax))

    # list of times
    ts = [float(i+1)*float(tf)/ticks for i in range(ticks)]

    # list of angles
    qs = [0.0] * len(ts)
    # list of velocities
    qds = [0.0] * len(ts)
    # print("[LSPB] \t ts:{}".format(ts))
    for i, t in enumerate(ts):
        # print(t)
        if t <= tb:
            # print("case 1")
            qs[i] = q0 + (0.5 * qddmax * t**2)
            qds[i] = qddmax * t
        elif t <= (tf - tb):
            # print("case 2")
            qs[i] = (qf + q0 - qdmax * tf) / 2 + qdmax * t
            qds[i] = qdmax
        else:
            # print("case 3")
            qs[i] = qf - (0.5 * qddmax * (t - tf)**2) 
            qds[i] = qddmax * (tf -  t)


    print("[LSPB] \t final qs:{}".format(qs))
    return ts, qs, qds, "ok"


def plot(ts, qs, qds, name="q"):
    plt.figure(figsize=(8,8))

    if type(name) == str:
        plt.title(name)
        plt.subplot(211)
        plt.plot(ts, qs, 'o-')
        plt.subplot(212)
        plt.plot(ts, qds, 'o-')
    elif type(name) == list:
        for (t, q, qd, n) in zip(ts, qs, qds, name):
            plt.subplot(211)
            plt.plot(t, q, 'o-', label="{}_pos".format(n))
            plt.subplot(212)
            plt.plot(t, qd, 'o-', label="{}_vel".format(n))
    plt.subplot(211)
    plt.legend(loc="center left",bbox_to_anchor=(1, 0.7))
    plt.subplot(212)
    plt.legend(loc="center left",bbox_to_anchor=(1, 0.7))
    plt.tight_layout()
    plt.show()

class MinimalPublisher(Node):

    def __init__(self, ts, qs, qds):
        super().__init__('joint_commander')

        # publisher for joint command
        self.publisher_ = self.create_publisher(JointTrajectory, '/arm_controller/joint_trajectory', 10)

        self.ts = ts
        self.qs = qs
        self.qds = qds

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
        points = []
        for t, q, qd in zip(ts, qs, qds):
            target_point = JointTrajectoryPoint()
            target_point.positions = [
                initial_state.position[self.joint_idx["arm_1_joint"]],
                initial_state.position[self.joint_idx["arm_2_joint"]],
                initial_state.position[self.joint_idx["arm_3_joint"]],
                initial_state.position[self.joint_idx["arm_4_joint"]],
                initial_state.position[self.joint_idx["arm_5_joint"]],
                initial_state.position[self.joint_idx["arm_6_joint"]],
                q
            ]
            target_point.velocities = [0.0] * 7
            target_point.velocities[-1] = qd
            target_point.accelerations = [0.0] * 7
            secs = int(t)
            target_point.time_from_start.sec = secs
            target_point.time_from_start.nanosec = int((t -secs) * 1e9) 
            points.append(target_point)
        msg.points = points
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
        # self.get_logger().info('Got: {}'.format(msg))

def ros_main(args, ts, qs, qds):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher(ts, qs, qds)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":

    # test
    ts, qs, qds, ok = lspb(
        2,  #tf
        0,  #q0
        2,  #qf
        max_speed[-1],  #qdmax
        5 #ticks
    )
    print("ok: {}".format(ok))
    print("ts: {}".format(ts))
    print("qs: {}".format(qs))
    print("qds: {}".format(qds))
    
    # execute for the first joint
    print(sys.argv)
    if len(sys.argv) > 1:
        if str(sys.argv[1]) == "exec":
            ros_main(None, ts, qs, qds)


    plot(ts,qs,qds)


