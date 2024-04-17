
# This script uses MoveIt functionalities for convenience 
# to visualise the trajectory on the rviz planning scene.
import rclpy
from rclpy.node import Node
import time
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState


last_state = None
joint_idx = {}
js_pub = None
publisher = None

def init(node):
    global last_state, joint_idx, js_pub, publisher
    # logger = rclpy.get_logger("viz_trajectory")
    publisher = node.create_publisher(DisplayTrajectory, '/display_planned_path', 10)

    # joints state callback
    def js_cb(msg):
        global last_state, joint_idx, js_pub
        if len(joint_idx) == 0:
            joint_idx = dict(zip(
                msg.name,
                [i for i in range(len(msg.name))]
            ))

        last_state = msg
    
    # subscriber to joint states
    js_pub = node.create_subscription(
        JointState,
        "/joint_states",
        js_cb,
        10
    )
    
    # period = 0.5
    # node.i = 0
    # timer = node.create_timer(period, node.display)  

def display(node, trajectory):
    if last_state is None:
        return 

    msg_disp_traj = DisplayTrajectory()
    msg_disp_traj.model_id = "tiago"
    msg_disp_traj.trajectory.append(RobotTrajectory())
    msg_disp_traj.trajectory_start = RobotState()
    msg_disp_traj.trajectory_start.joint_state = last_state
    msg_disp_traj.trajectory_start.is_diff = True

    msg_disp_traj.trajectory[0].joint_trajectory = trajectory

    publisher.publish(msg_disp_traj)


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('viz')
        # logger = rclpy.get_logger("viz_trajectory")


def main():
    rclpy.init()

    minimal_publisher = MinimalPublisher()

    init(minimal_publisher)

    print("waiting")
    while last_state is None:
        rclpy.spin_once(minimal_publisher)

    print("state received")

    joint_traj = JointTrajectory()
    joint_traj.joint_names = last_state.name
    joint_traj.points = []

    times = [t for t in range(10)]

    for i, t in enumerate(times):
        # define our  point message
        msg_point = JointTrajectoryPoint()
        msg_point.positions = last_state.position[:]
        msg_point.positions[joint_idx["arm_6_joint"]] = 0.3*i
        # msg_point.positions = [
        #     0.3*i,
        #     0.3*i,
        #     0.3*i,
        #     0.3*i,
        #     0.3*i,
        #     0.3*i,
        # ]
        msg_point.velocities = last_state.velocity[:]
        msg_point.velocities[joint_idx["arm_6_joint"]] = 0.1
        #  [
        #     1.,
        #     1.,
        #     1.,
        #     1.,
        #     1.,
        #     1.,
        #     1.
        # ]
        msg_point.accelerations = [0.] * len(last_state.velocity)
        secs = int(t)
        msg_point.time_from_start.sec = secs
        msg_point.time_from_start.nanosec = int((t -secs) * 1e9) 

        joint_traj.points.append(msg_point)

    display(minimal_publisher, joint_traj)

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()