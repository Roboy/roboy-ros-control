cmake_minimum_required(VERSION 2.8.3)
project(trajectory_advertiser)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

add_executable(trajectory_advertiser src/trajectory_advertiser.cpp)
target_link_libraries(trajectory_advertiser ${catkin_LIBRARIES})
