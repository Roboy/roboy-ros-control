cmake_minimum_required(VERSION 2.8.3)
project(myo_master)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

SET(CMAKE_BUILD_TYPE Debug)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp controller_manager std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/../common_utilities/include)

FILE(GLOB srcs "src/*.cpp")

add_executable(singleJoint src/singleJoint.cpp)
target_link_libraries(singleJoint ${catkin_LIBRARIES})

add_executable(multiJoint src/multiJoint.cpp src/roboy_hardwareInterface.cpp src/FlexRayHardwareInterface.cpp)

# dont forget this line, otherwise the messages will not be generated properly
add_dependencies(multiJoint common_utilities_generate_messages_cpp)

target_link_libraries(multiJoint ${catkin_LIBRARIES} -lftd2xx -lncurses)
