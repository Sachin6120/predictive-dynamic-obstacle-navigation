# A single persistent dynamic-obstacle track, expressed in the map frame.

uint32 id                          # stable track identifier, unique for the lifetime of the tracker

geometry_msgs/Point position       # published position: Kalman-filtered once kalman_initialized,
                                    # otherwise identical to raw_position (x, y; z unused, set to 0)
geometry_msgs/Vector3 velocity     # published velocity: Kalman-filtered once kalman_initialized,
                                    # otherwise the Stage-4B raw finite-difference velocity (vz unused)
float64 speed                      # hypot(vx, vy) of the published velocity, convenience field

geometry_msgs/Point raw_position   # Stage-4B measurement: this scan's associated cluster centroid,
                                    # unfiltered (x, y; z unused, set to 0)
geometry_msgs/Vector3 raw_velocity # Stage-4B baseline: raw finite-difference velocity from the
                                    # short position history (vz unused). Always populated so
                                    # RAW vs KALMAN comparison remains possible on every track.

bool kalman_initialized            # true once the per-track constant-velocity Kalman filter has
                                    # been corrected by >= 2 observations (a real velocity estimate
                                    # exists); false for a just-created track (position-only, v=0)

# Row-major 4x4 Kalman state covariance P for state [px, py, vx, vy]^T.
# Meaningless (identity-scaled prior) until kalman_initialized is true.
float64[16] covariance

uint32 age                         # number of scan cycles since track creation
uint32 observations                # number of scans in which this track was successfully associated
uint32 missed_count                # consecutive scan cycles since the last successful association

builtin_interfaces/Time stamp      # timestamp of the most recent observation used to update this track

# Future trajectory prediction (Stage-4C): state propagated forward from the
# current filtered state without modifying it. Empty until kalman_initialized.
TrackedObjectPrediction[] predictions
