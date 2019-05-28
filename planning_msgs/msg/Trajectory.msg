std_msgs/Header header

# Parametric trajectory represented as points that
# define cubic spiral + constant acceleration segments
planning_msgs/TrajectoryPoint[] points

# Status information
uint8 trajectory_status

# Status enum options
uint8 FAILED=0 # Failed to find a suitable trajectory
uint8 STITCHED=1 # Smoothly continues from a previous trajectory
uint8 REINIT=2 # Reinitialized from ego position, no continuity
uint8 IN_DEADZONE=3 # Didn't escape deadzone, is a stopping trajectory
