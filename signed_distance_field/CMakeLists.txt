cmake_minimum_required(VERSION 2.8.3)
project(signed_distance_field)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  abseil_cpp
  nav_msgs
  planning_msgs
  ros_utilities
  tf
  trajectory_math
)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS abseil_cpp nav_msgs planning_msgs ros_utilities tf trajectory_math
  DEPENDS OpenCV
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
 ${OpenCV_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers "include/*.h")
add_library(${PROJECT_NAME}
  src/signed_distance_field.cc
  src/signed_distance_field_introspection.cc
  ${headers}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(signed_distance_field_test test/signed_distance_field_test.cc)
  target_link_libraries(signed_distance_field_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

  catkin_add_gtest(signed_distance_field_generation_test test/signed_distance_field_generation_test.cc)
  target_link_libraries(signed_distance_field_generation_test ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})
endif()
