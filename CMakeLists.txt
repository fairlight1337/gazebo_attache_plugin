cmake_minimum_required(VERSION 2.8.3 FATAL_ERROR)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  roslib
  roscpp
  attache_msgs)

find_package(Boost REQUIRED COMPONENTS system)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

include (FindPkgConfig)
if (PKG_CONFIG_FOUND)
  pkg_check_modules(GAZEBO gazebo)
endif()

message(STATUS ${GAZEBO_LIBRARY_DIRS})

include_directories(
  include
  /usr/local/include/gazebo-2.2
  /usr/include/gazebo-2.2
  /usr/include/sdformat-1.4
  ${catkin_INCLUDE_DIRS})
  #${GAZEBO_INCLUDE_DIRS})

link_directories(
  /usr/local/lib/x86_64-linux-gnu/gazebo-2.2/plugins/
  ${catkin_LIBRARY_DIRS})
  #${GAZEBO_LIBRARY_DIRS})

add_library(attache SHARED
  src/Attache.cpp)

target_link_libraries(attache
  ${GAZEBO_LIBRARIES}
  ${Boost_LIBRARIES}
  ${catkin_LIBRARIES})
