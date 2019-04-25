cmake_minimum_required(VERSION 2.8.3)
project(waypoint_ui)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rviz
  tf)

set(CMAKE_AUTOMOC ON)

if(rviz_QT_VERSION VERSION_LESS "5")
  message(STATUS "Using Qt4 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt4 ${rviz_QT_VERSION} EXACT REQUIRED QtCore QtGui)
  ## pull in all required include dirs, define QT_LIBRARIES, etc.
  include(${QT_USE_FILE})
else()
  message(STATUS "Using Qt5 based on the rviz_QT_VERSION: ${rviz_QT_VERSION}")
  find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
  ## make target_link_libraries(${QT_LIBRARIES}) pull in all required dependencies
  set(QT_LIBRARIES Qt5::Widgets)
endif()

catkin_package()

add_definitions(-DQT_NO_KEYWORDS)
add_compile_options(-std=c++11)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${catkin_LIBRARY_DIRS})

add_library(waypoint_tool
  src/waypoint_tool.h
  src/waypoint_tool.cc
)
target_link_libraries(waypoint_tool
  ${QT_LIBRARIES}
  ${catkin_LIBRARIES}
)
