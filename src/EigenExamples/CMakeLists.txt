cmake_minimum_required( VERSION 3.28 )
enable_language(CXX)

set(CMAKE_CXX_STANDARD 11)

find_package(Pangolin REQUIRED)
find_package(glog REQUIRED)
find_package(gflags REQUIRED)
include_directories("/usr/include/eigen3")
include_directories(${Pangolin_INCLUDE_DIRS})

project(hello_slam)
add_executable(use_geometry use_geometry.cpp)
add_executable(coor_trans coordinate_transform.cpp)
add_executable(trajectory visualize_trajectory.cpp)

target_link_libraries(trajectory ${Pangolin_LIBRARIES})
target_link_libraries(trajectory glog::glog)
target_link_libraries(trajectory gflags)
