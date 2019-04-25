cmake_minimum_required(VERSION 2.8.3)
project(trajectory_math)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS abseil_cpp)
find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS abseil_cpp
  DEPENDS
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${EIGEN3_INCLUDE_DIRS}
)

file(GLOB_RECURSE headers "include/*.h")
add_library(${PROJECT_NAME}
  src/cubic_hermite_interpolation.cc
  src/cubic_spiral.cc
  src/curve_algorithms.cc
  src/gauss_legendre_integration.cc
  src/path_algorithms.cc
  src/point_algorithms.cc
  src/trajectory_algorithms.cc
  src/trajectory1D_algorithms.cc
  ${headers}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(curve_algorithms_test test/curve_algorithms_test.cc)
  target_link_libraries(curve_algorithms_test ${PROJECT_NAME})

  catkin_add_gtest(path_algorithms_test test/path_algorithms_test.cc)
  target_link_libraries(path_algorithms_test ${PROJECT_NAME})

  catkin_add_gtest(point_algorithms_test test/point_algorithms_test.cc)
  target_link_libraries(point_algorithms_test ${PROJECT_NAME})

  catkin_add_gtest(cubic_hermite_test test/cubic_hermite_test.cc)
  target_link_libraries(cubic_hermite_test ${PROJECT_NAME})

  catkin_add_gtest(gauss_legendre_test test/gauss_legendre_test.cc)
  target_link_libraries(gauss_legendre_test ${PROJECT_NAME})

  catkin_add_gtest(cubic_spiral_test test/cubic_spiral_test.cc)
  target_link_libraries(cubic_spiral_test ${PROJECT_NAME})
endif()
