cmake_minimum_required(VERSION 2.8.3)
project(hw_interface)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp controller_manager)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(singleJoint src/singleJoint.cpp)
target_link_libraries(singleJoint ${catkin_LIBRARIES})
