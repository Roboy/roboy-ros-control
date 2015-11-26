cmake_minimum_required(VERSION 2.8.3)
project(gazebo_control)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp controller_manager gazebo_ros )

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

link_directories(${GAZEBO_LIBRARY_DIRS})

## Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp gazebo_ros)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(singleJoint_gazebo src/singleJoint_gazebo.cpp)
target_link_libraries(singleJoint_gazebo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
