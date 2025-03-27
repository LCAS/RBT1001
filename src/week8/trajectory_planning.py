import rclpy
from rclpy.node import Node
from inverse_kinematics_analytic import compute_ik
from direct_kinematics import compute_dk
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt

class TrajectoryPlanning(Node):
    def __init__(self):
        super().__init__('traj_client')

    def plan_cartesian_trajectory(self):
        
        # Define waypoints in cartesian space (x, y, z)
        # NOTE: change at will (just make sure they are inside the robot's workspace)
        waypoints = [
            [0.1, 0.1, 0.2],
            [0.1, -0.1, 0.2],
            [-0.1, -0.1, 0.2]
        ]

        # Introduce 20 regularly distributed viapoints between waypoints
        # TODO: complete the code
        via_points = [waypoints[0]]

        # plot them for visualization
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot([p[0] for p in waypoints], [p[1] for p in waypoints], [p[2] for p in waypoints], 'ro-', markersize=12)
        ax.plot([p[0] for p in viapoints], [p[1] for p in viapoints], [p[2] for p in viapoints], 'bo-')
        plt.show()

        # set a fixed rotation for the end effector
        R = Rotation.from_matrix([
            [0, 0, 1],
            [0, 1, 0],
            [-1, 0, 0]
        ]).as_matrix()

        # for each pair of viapoints, calculate the joint states using IK
        # TODO: complete the code
        # HINT: use the compute_ik function from inverse_kinematics_analytic.py
        qs = []

        # for each joint, plot the trajectory with waypoints and via points
        for i in range(6):
            plt.plot([q[i] for q in qs], label='joint' + str(i + 1))
        plt.legend()
        plt.show()   

    def plan_joint_trajectory(self):

        # Define waypoints in joint space 
        # NOTE: change at will
        waypoints = [
            [0., 0., 0., 0., 0., 0.],
            [0.1, 0.2, 0.3, 0.4, 0.5, 0.6],
            [1.5, 1.4, 1.3, 1.2, 1.1, 1.]
        ]

        # Introduce 20 regularly distributed viapoints between waypoints
        # TODO: complete the code
        viapoints = [waypoints[0]]

        
        # plot them for visualization
        fig = plt.figure()
        ax = fig.add_subplot(111)
        for i in range(6):
            ax.plot([0, 20, 40], [p[i] for p in waypoints],'yo-', markersize=12)
            ax.plot([p[i] for p in viapoints], 'o-', label='joint' + str(i + 1))
        ax.legend()
        plt.show()

        # for each pair of viapoints, calculate the end effector position using compute_dk function
        # TODO: complete the code
        # HINT: use the compute_dk function from direct_kinematics.py
        
        # plot the trajectory in cartesian space
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.plot([p[0] for p in Ps], [p[1] for p in Ps], [p[2] for p in Ps], 'bo-')
        plt.show()


if __name__ == '__main__':
    rclpy.init()
    node = TrajectoryPlanning()
    node.plan_cartesian_trajectory()
    node.plan_joint_trajectory()
    rclpy.spin(node)
    rclpy.shutdown()
