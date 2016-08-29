cmake_minimum_required(VERSION 2.8.3)
project(myo_master)

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../flexrayusbinterface/cmake)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/devel/${PROJECT_NAME})

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g")
SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS 
				roscpp 
				controller_manager 
				std_msgs
				common_utilities
				flexrayusbinterface)

find_package(Boost REQUIRED COMPONENTS thread)

include_directories(
	include 
	${catkin_INCLUDE_DIRS}
	${Boost_INCLUDE_DIRS} 

)

catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp message_runtime controller_manager std_msgs common_utilities flexrayusbinterface
   DEPENDS Boost)
#==============================================================================
# Application
#==============================================================================
add_executable(multiJoint 
		src/main.cpp 
		src/roboy.cpp 
)

add_dependencies(multiJoint 
					${catkin_EXPORTED_TARGETS}
					common_utilities_gencpp
					${flexrayusbinterface_EXPORTED_TARGETS}
)

target_link_libraries(multiJoint 
						${catkin_LIBRARIES} 
						${Boost_LIBRARIES} 
						-lftd2xx
						-ldl 
						-lpthread)

#==============================================================================
# Installation
#==============================================================================
install( TARGETS    	
	multiJoint
	DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES 
	package.xml
  	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY 
	launch/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY 
	worlds/
	DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
