cmake_minimum_required(VERSION 2.8.3)
project(roboy_controller)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

SET(CMAKE_BUILD_TYPE Debug)

#==============================================================================
# Dependencies
#==============================================================================
find_package(catkin REQUIRED COMPONENTS roscpp controller_interface hardware_interface pluginlib)
catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/../common_utilities/include
        ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include
)

#==============================================================================
# Libraries
#==============================================================================
add_library(PositionController src/PositionController.cpp)
add_dependencies(PositionController common_utilities_generate_messages_cpp)
target_link_libraries(PositionController ${catkin_LIBRARIES})

add_library(VelocityController src/VelocityController.cpp)
add_dependencies(VelocityController common_utilities_generate_messages_cpp)
target_link_libraries(VelocityController ${catkin_LIBRARIES})

add_library(ForceController src/ForceController.cpp)
add_dependencies(ForceController common_utilities_generate_messages_cpp)
target_link_libraries(ForceController ${catkin_LIBRARIES})

add_library(MuscleActivityController src/MuscleActivityController.cpp)
add_dependencies(MuscleActivityController common_utilities_generate_messages_cpp)
target_link_libraries(MuscleActivityController ${catkin_LIBRARIES})

#==============================================================================
# Installation
#==============================================================================
install(TARGETS
        PositionController
        VelocityController
        ForceController
        MuscleActivityController
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        )

install(FILES
        package.xml
        PositionController_plugin.xml
        VelocityController_plugin.xml
        ForceController_plugin.xml
        MuscleActivityController_plugin.xml
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        )

install(DIRECTORY
        config/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
        )
