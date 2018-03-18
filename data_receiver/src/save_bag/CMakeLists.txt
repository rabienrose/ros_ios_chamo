cmake_minimum_required(VERSION 2.8.3)
project(save_bag)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -ggdb")

include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
include_directories( ${CMAKE_CURRENT_SOURCE_DIR}/include)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  std_msgs
  message_generation
)
IF(DEFINED ENV{DEV_BUILD})
  ADD_DEFINITIONS(-DDEV_BUILD) 
ENDIF()

add_message_files(DIRECTORY msg
FILES  
img_chamo.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

catkin_package(CATKIN_DEPENDS message_runtime)

find_package(OpenCV REQUIRED)
message(STATUS ${catkin_INCLUDE_DIRS})
INCLUDE_DIRECTORIES(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})

add_executable(save_bag src/main.cpp)
target_link_libraries(save_bag ${catkin_LIBRARIES} ${OpenCV_LIBS})
set(CATKIN_PACKAGE_BIN_DESTINATION bin/)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
 )


