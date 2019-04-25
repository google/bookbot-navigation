# Bookbot Navigation

This collection of ROS packages represents a simple motion planning stack for a library book delivery robot. This planning stack consists of three primary planning nodes:
1. `primitive_planner` is a path planner that performs an A* search through a tree of motion primitives.
2. `primitive_velocity_planner` is a velocity profile planner that plans an optimal velocity profile for a specified path. This node also performs an A* search through a tree of velocity profile primitives.
3. `trajectory_following_controller` is a simple trajectory following controller that uses pure pursuit to compute control commands that keep the robot following its planned trajectory.

In addition an rviz-based waypoint UI and a simple simulation node are also provided to allow for out-of-the-box simulation. 

## Basic Usage

To try out the bookbot navigation planner:
1. Compile the packages in a ROS workspace (and resource the setup.bash for the workspace to ensure rviz can find the waypoint tool)
2. Load the `primitive_velocity_planner/rviz/planner.rviz` config in rviz
3. Run the planning stack in simulation with a static example obstacle grid: 
	`roslaunch primitive_velocity_planner test_primitive_velocity_planner.launch`
4. Set new waypoints by clicking on the waypoint tool (or using `w` as a hotkey) and then `shift-click` to append a new waypoint or `ctrl-click` to replace the existing set of waypoints.
