cmake_minimum_required(VERSION 2.8.3)
project(roboy_simulation)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(gazebo REQUIRED)
include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})

find_package(catkin REQUIRED COMPONENTS roscpp controller_manager gazebo_ros_control std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
	${catkin_INCLUDE_DIRS} 
	include/ 
	${CMAKE_CURRENT_SOURCE_DIR}/../common_utilities/include
	${CMAKE_CURRENT_SOURCE_DIR}/../flexrayusbinterface/include
)
#==============================================================================
# Application
#==============================================================================
add_library(muscle_plugin SHARED src/MusclePlugin.cc)
target_link_libraries(muscle_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} ${ARMADILLO_LIBRARIES} ${catkin_LIBRARIES})

add_library(tendon_plugin SHARED src/TendonPlugin.cc)
target_link_libraries(tendon_plugin ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})

add_library(actuator_plugin SHARED src/ActuatorPlugin.cc)
target_link_libraries(actuator_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${ARMADILLO_LIBRARIES} )

add_library(roboySim SHARED src/roboySim.cpp)
add_dependencies(roboySim common_utilities_gencpp)
target_link_libraries(roboySim ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES})
