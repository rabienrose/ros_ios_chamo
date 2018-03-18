cmake_minimum_required(VERSION 2.8.3)
project(rostime)
set(Boost_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/boost/include)
set(catkin_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})
add_library(rostime 
  src/duration.cpp
  src/rate.cpp
  src/time.cpp)