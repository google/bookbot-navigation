cmake_minimum_required(VERSION 2.8.3)
project(ros_utilities)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS abseil_cpp diagnostic_msgs roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS abseil_cpp diagnostic_msgs roscpp
  DEPENDS
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers "include/*.h")
add_library(${PROJECT_NAME}
  src/introspection.cpp
  src/scoped_timer.cpp
  ${headers}
)

if (CATKIN_ENABLE_TESTING)
  find_package(std_msgs REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(rostest REQUIRED)
  add_rostest_gtest(topic_listener_test
                    test/topic_listener_test.launch
                    test/topic_listener_test.cpp)
  target_link_libraries(topic_listener_test ${catkin_LIBRARIES})
endif()
