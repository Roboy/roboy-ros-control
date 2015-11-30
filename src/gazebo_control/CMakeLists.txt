cmake_minimum_required(VERSION 2.8.3)
project(gazebo_control)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp controller_manager gazebo_ros control_toolbox)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED gazebo_physics gazebo_sensors)

FIND_LIBRARY(DEFAULT_ROBOT_HW_SIM_LIBRARY default_robot_hw_sim
	/opt/ros/indigo/lib
	NO_DEFAULT_PATH
)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS roscpp gazebo_ros)

include_directories(include ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_executable(singleJoint_gazebo src/singleJoint_gazebo.cpp)
target_link_libraries(singleJoint_gazebo ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${DEFAULT_ROBOT_HW_SIM_LIBRARY} -lgazebo_sensors)
