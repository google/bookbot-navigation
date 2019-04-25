cmake_minimum_required(VERSION 2.8.3)
project(bookbot_base_simulator)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  abseil_cpp
  cmake_modules
  geometry_msgs
  nav_msgs
  roscpp
  tf
  tf_conversions
)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS abseil_cpp geometry_msgs roscpp tf tf_conversions
  DEPENDS
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${Eigen_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers "include/*.h")
add_library(${PROJECT_NAME}
  src/robot_motion_model.cpp
  src/bookbot_base_simulator.cpp
  ${headers}
)

add_executable(${PROJECT_NAME}_node src/bookbot_base_simulator_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
