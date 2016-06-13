cmake_minimum_required(VERSION 2.8.3)
project(myo_master)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/${PROJECT_NAME})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp controller_manager std_msgs)
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
add_executable(multiJoint src/main.cpp src/roboy.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../flexrayusbinterface/src/FlexRayHardwareInterface.cpp)
add_dependencies(multiJoint common_utilities_gencpp)
target_link_libraries(multiJoint ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} ${Boost_LIBRARIES} -lftd2xx)
