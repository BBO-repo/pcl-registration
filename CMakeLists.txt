cmake_minimum_required(VERSION 3.10)
project(pcl_registration)

add_definitions(-std=c++14)
set(CMAKE_CXX_FLAGS "-std=c++14")

# pcl 1.7 causes a segfault when it is built with debug mode
set(CMAKE_BUILD_TYPE "RELEASE")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

message(STATUS "PCL_INCLUDE_DIRS:" ${PCL_INCLUDE_DIRS})
message(STATUS "PCL_LIBRARY_DIRS:" ${PCL_LIBRARY_DIRS})
message(STATUS "PCL_DEFINITIONS:" ${PCL_DEFINITIONS})

###########
## Build ##
###########
add_executable(align
  src/align.cpp
)

target_link_libraries(align
  ${PCL_LIBRARIES}
)
