cmake_minimum_required(VERSION 2.8.3)
project(gui_dummy)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

## Declare a catkin package
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(include ${catkin_INCLUDE_DIRS})

include_directories(include "include/")

add_executable(gui_dummy src/main.cpp src/gui_dummy.cpp)

# dont forget this line, otherwise the messages will not be generated properly
add_dependencies(gui_dummy common_utilities_generate_messages_cpp)

target_link_libraries(gui_dummy ${catkin_LIBRARIES} -lncurses)
