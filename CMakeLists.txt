cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS} ${script})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(ActorPLugin SHARED script/ActorPLugin.cc script/ActorPLugin.hh)
target_link_libraries(ActorPLugin ${GAZEBO_LIBRARIES})
