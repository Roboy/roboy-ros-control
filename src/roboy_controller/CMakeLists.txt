cmake_minimum_required(VERSION 2.8.3)
project(roboy_controller)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

SET(CMAKE_BUILD_TYPE Debug)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp controller_interface hardware_interface pluginlib)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
include 
${catkin_INCLUDE_DIRS}
${CMAKE_CURRENT_SOURCE_DIR}/../common_utilities/include
)

############ POSITION CONTROLLER #############
add_library(PositionController src/PositionController.cpp)
# dont forget this line, otherwise the messages will not be generated properly
add_dependencies(PositionController common_utilities_generate_messages_cpp)
target_link_libraries(PositionController ${catkin_LIBRARIES})

############ VELOCITY CONTROLLER #############
add_library(VelocityController src/VelocityController.cpp)
# dont forget this line, otherwise the messages will not be generated properly
add_dependencies(VelocityController common_utilities_generate_messages_cpp)
target_link_libraries(VelocityController ${catkin_LIBRARIES})

############ FORCE CONTROLLER #############
add_library(ForceController src/ForceController.cpp)
# dont forget this line, otherwise the messages will not be generated properly
add_dependencies(ForceController common_utilities_generate_messages_cpp)
target_link_libraries(ForceController ${catkin_LIBRARIES})
