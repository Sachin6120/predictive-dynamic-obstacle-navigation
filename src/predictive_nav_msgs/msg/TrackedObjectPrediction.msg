# A single future state estimate for one TrackedObject, produced by
# propagating the track's constant-velocity Kalman filter forward without
# modifying its live state. Stage-4C evaluation output; also the intended
# input shape for a future (Stage-4D, not implemented here) predictive
# costmap layer.

float64 time_from_now                # seconds after the parent TrackedObject.stamp
builtin_interfaces/Time stamp        # absolute predicted timestamp (parent stamp + time_from_now)

geometry_msgs/Point position         # predicted x, y in map frame (z unused, set to 0)
geometry_msgs/Vector3 velocity       # predicted vx, vy (constant-velocity model; vz unused)

# Row-major 2x2 position covariance block of the propagated Kalman state:
# [ xx, xy, yx, yy ]. Grows with time_from_now; the sole basis for the
# uncertainty visualization and for any future conservative-occupancy use.
float64[4] position_covariance
