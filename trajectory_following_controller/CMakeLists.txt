cmake_minimum_required(VERSION 2.8.3)
project(trajectory_following_controller)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  nav_msgs
  planning_msgs
  ros_utilities
  roscpp
  tf
  trajectory_math
  visualization_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    geometry_msgs
    nav_msgs
    planning_msgs
    ros_utilities
    roscpp
    tf
    trajectory_math
    visualization_msgs
  DEPENDS
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers "include/*.h")
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cc
  src/trajectory_following_controller_introspection.cc
  ${headers}
)

add_executable(${PROJECT_NAME}_node
  src/${PROJECT_NAME}_node.cc
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)