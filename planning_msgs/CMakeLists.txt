cmake_minimum_required(VERSION 2.8.3)
project(planning_msgs)
find_package(catkin REQUIRED COMPONENTS message_generation geometry_msgs std_msgs )

# Generate messages in the 'msg' folder
add_message_files(
  FILES
  Path.msg
  PathPoint.msg
  Trajectory.msg
  TrajectoryPoint.msg
  WaypointList.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(CATKIN_DEPENDS std_msgs geometry_msgs)
