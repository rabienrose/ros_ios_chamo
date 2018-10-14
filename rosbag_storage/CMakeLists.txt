cmake_minimum_required(VERSION 2.8.3)
project(rosbag_storage)
set(Console_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/console_bridge/include ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/console_bridge/build)
set(Boost_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../thirdparty/boost/include)
set(catkin_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(${Console_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS})

add_library(rosbag_storage
  ${AES_ENCRYPT_SOURCE}
  src/bag.cpp
  src/bag_player.cpp
  src/buffer.cpp
  #src/bz2_stream.cpp
  #src/lz4_stream.cpp
  src/chunked_file.cpp
  src/encryptor.cpp
  src/message_instance.cpp
  src/query.cpp
  src/stream.cpp
  src/view.cpp
  src/uncompressed_stream.cpp
)
