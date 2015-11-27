cmake_minimum_required(VERSION 2.8.3)
project(hw_controller)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp controller_interface hardware_interface pluginlib)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(singleJoint_controller src/singleJoint_controller.cpp)
target_link_libraries(singleJoint_controller ${catkin_LIBRARIES})
