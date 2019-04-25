cmake_minimum_required(VERSION 2.8.3)
project(primitive_planner)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  planning_msgs
  roscpp
  ros_utilities
  signed_distance_field
  tf
  trajectory_math
  visualization_msgs
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
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
  src/primitive.cc
  src/primitive_planner_introspection.cc
  src/primitive_search_node.cc
  src/visited_grid.cc
  ${headers}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)