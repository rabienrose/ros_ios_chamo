cmake_minimum_required(VERSION 2.8.3)
project(xmlrpcpp)
set(catkin_INCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/../include)
include_directories(include ${catkin_INCLUDE_DIRS})
add_library(xmlrpcpp
  src/XmlRpcClient.cpp
  src/XmlRpcDispatch.cpp
  src/XmlRpcServer.cpp
  src/XmlRpcServerConnection.cpp
  src/XmlRpcServerMethod.cpp
  src/XmlRpcSocket.cpp
  src/XmlRpcSource.cpp
  src/XmlRpcUtil.cpp
  src/XmlRpcValue.cpp
  )