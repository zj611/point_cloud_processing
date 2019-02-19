cmake_minimum_required(VERSION 3.12)
project(ICP_PCL_demo)

set(CMAKE_CXX_STANDARD 11)

FIND_PACKAGE(PCL 1.5 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


add_executable(icp iterative_closeast_point.cpp )
target_link_libraries (icp ${PCL_LIBRARIES})
