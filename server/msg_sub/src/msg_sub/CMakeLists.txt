cmake_minimum_required(VERSION 2.8.3)
project(msg_sub)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  std_msgs
  sensor_msgs
  geometry_msgs
)
find_package(OpenCV REQUIRED QUIET)


include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${OpenCV_INCLUDE_DIRS}
)

link_directories(
	include
	${OpenCV_LIBRARY_DIRS}
)

add_executable(msg_sub src/main.cpp)
target_link_libraries(msg_sub ${OpenCV_LIBRARIES} ${catkin_LIBRARIES})
