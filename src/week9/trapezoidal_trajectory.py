import rclpy
from rclpy.node import Node
# from inverse_kinematics_analytic import compute_ik
# from direct_kinematics import compute_dk
from scipy.spatial.transform import Rotation
import matplotlib.pyplot as plt
import numpy as np

class TrajectoryPlanning(Node):
    def __init__(self):
        super().__init__('traj_client')
    
    def plan_cartesian_trajectory(self):
        
        # Define waypoints in cartesian space (x, y, z)
        # NOTE: change at will (just make sure they are inside the robot's workspace)
        waypoints = [
            [0.1, 0.1, 0.0],
            [0.1, 0.4, 0.2],
            [-0.2, 0.6, 0.2]
        ]

        # set maximum cartesian velocity and time of execution
        pdmax = .2 # m/s
        time = 2 # seconds

        # Compute the trapezoidal trajectory for each of the dimensions separately (X, Y and Z)
        # with 20 intermediate points. Repeat for each pair of waypoints
        txs, pxs, pdxs = [], [], []
        tys, pys, pdys = [], [], []
        tzs, pzs, pdzs = [], [], []
        starting_time = 0.
        ticks = 20
        wp_idxs = [0]
        for init_point_idx in range(len(waypoints) - 1):
            print("Processing segment from waypoint {} to waypoint {}".format(init_point_idx, init_point_idx + 1))
            # compute the trapezoidal trajectory for each dimension
            print("Dim X")
            px0 = waypoints[init_point_idx][0] # initial point x
            pxf = waypoints[init_point_idx+1][0] # final point x
            _txs, _pxs, _pdxs, _newtimex = self.generate_trapezoidal_segment(time, px0, pxf, pdmax, ticks=ticks)
            txs += [t+starting_time for t in _txs]
            pxs += _pxs
            pdxs += _pdxs

            print("Dim Y")
            py0 = waypoints[init_point_idx][1] # initial point y
            pyf = waypoints[init_point_idx+1][1] # final point y
            _tys, _pys, _pdys, _newtimey = self.generate_trapezoidal_segment(time, py0, pyf, pdmax, ticks=ticks)
            tys += [t+starting_time for t in _tys]
            pys += _pys
            pdys += _pdys

            print("Dim Z")
            pz0 = waypoints[init_point_idx][2] # initial point z
            pzf = waypoints[init_point_idx+1][2] # final point z
            _tzs, _pzs, _pdzs, _newtimez = self.generate_trapezoidal_segment(time, pz0, pzf, pdmax, ticks=ticks)
            tzs += [t+starting_time for t in _tzs]
            pzs += _pzs
            pdzs += _pdzs

            wp_idxs += [len(txs) - 1]
            
            starting_time += max(_newtimex, _newtimey, _newtimez)

        # fix in case any trajectory was extended in the last segment
        # this is a workaround to make the plots aligned. 
        if txs[-1] < starting_time:
            txs.append(starting_time)
            pxs.append(pxs[-1])
            pdxs.append(pdxs[-1])
        if tys[-1] < starting_time:
            tys.append(starting_time)
            pys.append(pys[-1])
            pdys.append(pdys[-1])
        if tzs[-1] < starting_time:
            tzs.append(starting_time)
            pzs.append(pzs[-1])
            pdzs.append(pdzs[-1])

        # plot all the position and velocity trajectories for the three dimensions
        fig, axs = plt.subplots(3, 2, figsize=(12, 8))
        fig.suptitle('Trapezoidal Trajectory')
        axs[0, 0].set_title('X')
        axs[0, 0].set_xlabel('time [s]')
        axs[0, 0].set_ylabel('position [m]')
        axs[0, 1].set_title('X')
        axs[0, 1].set_xlabel('time [s]')
        axs[0, 1].set_ylabel('velocity [m/s]')
        axs[1, 0].set_title('Y')
        axs[1, 0].set_xlabel('time [s]')
        axs[1, 0].set_ylabel('position [m]')
        axs[1, 1].set_title('Y')
        axs[1, 1].set_xlabel('time [s]')
        axs[1, 1].set_ylabel('velocity [m/s]')
        axs[2, 0].set_title('Z')
        axs[2, 0].set_xlabel('time [s]')
        axs[2, 0].set_ylabel('position [m]')
        axs[2, 1].set_title('Z')
        axs[2, 1].set_xlabel('time [s]')
        axs[2, 1].set_ylabel('velocity [m/s]')
        axs[0, 0].plot(txs, pxs,'ro-', markersize=2)
        axs[0, 1].plot(txs, pdxs, 'bo-', markersize=2)
        axs[1, 0].plot(tys, pys, 'ro-', markersize=2)
        axs[1, 1].plot(tys, pdys,'bo-', markersize=2)
        axs[2, 0].plot(tzs, pzs, 'ro-', markersize=2)
        axs[2, 1].plot(tzs, pdzs, 'bo-', markersize=2)
        # add the waypoints to position plots and corresponding vertical lines to velocities plots
        for i in range(len(waypoints)):
            axs[0, 0].plot(txs[wp_idxs[i]], waypoints[i][0], 'go', markersize=7)
            axs[1, 0].plot(tys[wp_idxs[i]], waypoints[i][1], 'go', markersize=7)
            axs[2, 0].plot(tzs[wp_idxs[i]], waypoints[i][2], 'go', markersize=7)
            axs[0, 1].axvline(x=txs[wp_idxs[i]], color='g', linestyle='--')
            axs[1, 1].axvline(x=tys[wp_idxs[i]], color='g', linestyle='--')
            axs[2, 1].axvline(x=tzs[wp_idxs[i]], color='g', linestyle='--')

        plt.tight_layout()
        plt.show()


    # Linear Segments with Parabolic Blends
    # Computes the lspb along one dimention (X, Y or Z)
    #  tf:    the final time [secs] - int
    #  p0:    initial position [m] - float
    #  pf:    final position [m] - float
    #  pdmax: maximum velocity [m/s] - float
    #  ticks: number of subdivision points in the trajectory - int
    # Returns:
    #  Ts: corresponding list of times - list of float
    #  Ps: list of position values defined along the trajectory - list of float
    #  Pds: list of velocities - list of float
    def generate_trapezoidal_segment(self, tf, p0, pf, pdmax, ticks):
        
        print("[LSPB] Input data\n tf:{}, p0:{}, pf:{}, pdmax:{}, ticks:{}".format(tf, p0, pf, pdmax, ticks))

        # list of times
        ts = [0.] + [float(i+1)*float(tf)/ticks for i in range(ticks)]

        if abs(p0 - pf) <= 1e-6:
            print("[LSPB] \t p0 and pf are equal")
            return ts, [p0]*(ticks+1), [0.]*(ticks+1), tf

        # adjust the velocity sign based on the traj direction (i.e. velocity sign might need to be negative)
        pdmax = abs(pdmax) * np.sign(pf - p0)


        ## TODO: check that the velocity is not too high for the given time
        #        which means that we need to use a triangular profile instead
        trapezoidal = True
        if ## TODO
            print("We cannot reach max velocity in the given time, using triangular profile")
            trapezoidal = False

        if trapezoidal:
            ## TODO: find tb
            tb = ## TODO
            print("[LSPB] \t tb:{}".format(tb))

            ## TODO: find max acceleration
            pddmax = ## TODO
            print("[LSPB] \t pddmax:{}".format(pddmax))

            # list of angles
            ps = [0.0] * len(ts)
            # list of velocities
            pds = [0.0] * len(ts)
            print("[LSPB] \t ts:{}".format(ts))
            for i, t in enumerate(ts):
                ## TODO add the trajectory computation for the three segments
                #       - compute both position and velocity 
        else:
            ## NOTE: this is the case when we have a triangular profile

            ## TODO: find tb
            tb = ## TODO
            print("[LSPB] \t tb:{}".format(tb))

            # TODO: find max acceleration
            pddmax = ## TODO
            print("[LSPB] \t pddmax:{}".format(pddmax))

            # list of angles
            ps = [0.0] * len(ts)
            # list of velocities
            pds = [0.0] * len(ts)
            print("[LSPB] \t ts:{}".format(ts))
            for i, t in enumerate(ts):
                ## TODO add the trajectory computation for the two segments
                #      - compute both position and velocity

        print("[LSPB] \t ps:{}".format(ps))
        print("[LSPB] \t pds:{}".format(pds))

        return ts, ps, pds, tf





if __name__ == '__main__':
    rclpy.init()
    node = TrajectoryPlanning()
    node.plan_cartesian_trajectory()
    rclpy.spin(node)
    rclpy.shutdown()
