import numpy as np

# max speeds for each joint
max_speed = [
    18.0,
    18.0,
    22.0,
    22.0,
    17.0,
    17.0,
    17.0
]
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
#  qs: list of joint values defined along the trajectory
#  ts: corresponding list of times
#  qds: joint velocities
def lspb(tf, q0, qf, qdmax, ticks):

    return #

def plot(qs, ts, qds):

    