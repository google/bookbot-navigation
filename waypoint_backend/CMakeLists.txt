cmake_minimum_required(VERSION 2.8.3)
project(waypoint_backend)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  nav_msgs
  planning_msgs
  roscpp
  ros_utilities
  std_msgs
)
find_package(Eigen3 REQUIRED)
catkin_package()

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers "include/*.h")
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cc
  ${headers}
)

add_executable(${PROJECT_NAME}_node src/waypoint_backend_node.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
