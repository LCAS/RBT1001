
# This script uses MoveIt functionalities for convenience 
# to visualise the trajectory on the rviz planning scene.
import rclpy
from rclpy.node import Node
from moveit_msgs.msg import DisplayTrajectory, RobotTrajectory, RobotState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('viz')
        # logger = rclpy.get_logger("viz_trajectory")
        self.publisher = self.create_publisher(DisplayTrajectory, '/display_planned_path', 10)

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
        self.timer = self.create_timer(period, self.display)


    def display(self):
        if self.last_state is None or self.i > 0:
            return 

        msg_disp_traj = DisplayTrajectory()
        msg_disp_traj.model_id = "tiago"
        msg_disp_traj.trajectory.append(RobotTrajectory())
        msg_disp_traj.trajectory_start = RobotState()
        msg_disp_traj.trajectory_start.joint_state = self.last_state
        msg_disp_traj.trajectory_start.is_diff = True

        joint_traj = JointTrajectory()
        joint_traj.joint_names = self.last_state.name
        joint_traj.points = []

        times = [t for t in range(10)]

        for i, t in enumerate(times):
            # define our  point message
            msg_point = JointTrajectoryPoint()
            msg_point.positions = self.last_state.position[:]
            msg_point.positions[self.joint_idx["arm_6_joint"]] = 0.3*i
            # msg_point.positions = [
            #     0.3*i,
            #     0.3*i,
            #     0.3*i,
            #     0.3*i,
            #     0.3*i,
            #     0.3*i,
            #     0.3*i
            # ]
            msg_point.velocities = self.last_state.velocity[:]
            msg_point.velocities[self.joint_idx["arm_6_joint"]] = 0.1
            #  [
            #     1.,
            #     1.,
            #     1.,
            #     1.,
            #     1.,
            #     1.,
            #     1.
            # ]
            msg_point.accelerations = self.last_state.effort
            secs = int(t)
            msg_point.time_from_start.sec = secs
            msg_point.time_from_start.nanosec = int((t -secs) * 1e9) 

            joint_traj.points.append(msg_point)

        # print(self.last_state.position[self.joint_idx["arm_6_joint"]])
        # for i, t in enumerate(times):
        #     print(joint_traj.points[i].positions[self.joint_idx["arm_6_joint"]] )
        #     joint_traj.points[i].positions[self.joint_idx["arm_6_joint"]] = 0.
        #     print(joint_traj.points[i].positions[self.joint_idx["arm_6_joint"]] )

        msg_disp_traj.trajectory[0].joint_trajectory = joint_traj

        self.publisher.publish(msg_disp_traj)
        self.i += 1

    # joints state callback
    def js_cb(self, msg):

        if len(self.joint_idx) == 0:
            self.joint_idx = dict(zip(
                msg.name,
                [i for i in range(len(msg.name))]
            ))

        self.last_state = msg

def main():
    rclpy.init()

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()