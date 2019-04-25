cmake_minimum_required(VERSION 2.8.3)
project(path_converter)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
 nav_msgs
 planning_msgs
 roscpp
 tf
 trajectory_math
)
find_package(Eigen3 REQUIRED)

catkin_package()

include_directories(
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)

target_link_libraries(${PROJECT_NAME}_node
 ${catkin_LIBRARIES}
)
