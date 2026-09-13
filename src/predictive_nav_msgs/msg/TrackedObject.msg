# A single persistent dynamic-obstacle track, expressed in the map frame.

uint32 id                          # stable track identifier, unique for the lifetime of the tracker

geometry_msgs/Point position       # x, y in map frame (z is unused, set to 0)
geometry_msgs/Vector3 velocity     # vx, vy estimated from observation history (vz unused)
float64 speed                      # hypot(vx, vy), convenience field

uint32 age                         # number of scan cycles since track creation
uint32 observations                # number of scans in which this track was successfully associated
uint32 missed_count                # consecutive scan cycles since the last successful association

builtin_interfaces/Time stamp      # timestamp of the most recent observation used to update this track
