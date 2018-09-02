cmake_minimum_required(VERSION 2.8.3)
project(titan_arm)

find_package(catkin REQUIRED COMPONENTS
	roscpp
	roslaunch
	controller_manager
        actionlib
	titan_msgs
	hardware_interface
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS system chrono)

catkin_package(
#	INCLUDE_DIRS include
#  LIBRARIES titan_hardware
	CATKIN_DEPENDS roscpp control_toolbox hardware_interface std_msgs titan_msgs actionlib
#  DEPENDS system_lib
)

include_directories(
	include
	${Boost_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)

add_executable(titan_arm_action_server_node src/titan_arm_action_server.cpp
					    src/titan_arm_hardware.cpp	
	      )

add_dependencies(titan_arm_action_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(titan_arm_action_server_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
 )


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
