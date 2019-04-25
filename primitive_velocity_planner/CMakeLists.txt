cmake_minimum_required(VERSION 2.8.3)
project(primitive_velocity_planner)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  nav_msgs
  planning_msgs
  ros_utilities
  roscpp
  signed_distance_field
  tf
  trajectory_math
  visualization_msgs
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    planning_msgs
    ros_utilities
    roscpp
    signed_distance_field
    tf
    trajectory_math
    visualization_msgs
  DEPENDS
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers "include/*.h")
add_library(${PROJECT_NAME}
  src/primitive_velocity_planner.cc
  src/velocity_planner_introspection.cc
  src/velocity_profile_search.cc
  src/velocity_search_visited_grid.cc
  ${headers}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cc)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)
