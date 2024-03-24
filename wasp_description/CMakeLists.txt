cmake_minimum_required(VERSION 3.0.2)
project(wasp_description)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  geometry_msgs
  mavros_msgs
  tf
  robot_localization
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  RequestMission.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  mavros_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES wasp_description
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(wasp_vio src/VIOInterface.cpp)
target_link_libraries(wasp_vio ${catkin_LIBRARIES})
add_dependencies(wasp_vio wasp_description_gencpp)

add_executable(wasp_control src/SetpointControl.cpp)
target_link_libraries(wasp_control ${catkin_LIBRARIES})
add_dependencies(wasp_control wasp_description_gencpp)

add_executable(wasp_extruder src/Extruder.cpp)
target_link_libraries(wasp_extruder ${catkin_LIBRARIES})
add_dependencies(wasp_extruder wasp_description_gencpp)

add_executable(mission_server src/MissionServer.cpp)
target_link_libraries(mission_server ${catkin_LIBRARIES})
add_dependencies(mission_server wasp_description_gencpp)


install(PROGRAMS scripts/setup.sh DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
