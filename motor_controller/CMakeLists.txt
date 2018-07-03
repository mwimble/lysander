
cmake_minimum_required(VERSION 2.8.3)
project(motor_controller)

set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread" )
SET(FLAGS "-g -ggdb -g3")

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_generation
  nav_msgs
  rosconsole
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(Boost REQUIRED COMPONENTS system thread)

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  RoboClawStatus.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

generate_dynamic_reconfigure_options( cfg/motor_controller.cfg)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS dynamic_reconfigure geometry_msgs message_runtime rosconsole roscpp rospy sensor_msgs std_msgs tf
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable (
  WRDifferentialDrive_node
  src/WRDifferentialDrive_node.cpp
  src/WRDifferentialDrive.cpp
)

add_dependencies(
  WRDifferentialDrive_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(
  WRDifferentialDrive_node
  boost_system boost_thread
  ${catkin_LIBRARIES} 
)

add_executable(motor_controller_node src/motorController.cpp src/motorSkidSteerDrive.cpp)
add_executable(square_node src/squareTest.cpp)
add_executable(trace_rotation_node src/traceRotation.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(
  motor_controller_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  square_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)

add_dependencies(
  trace_rotation_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
)


## Specify libraries to link a library or executable target against
target_link_libraries(motor_controller_node
  ${catkin_LIBRARIES} 
)

target_link_libraries(square_node
  ${catkin_LIBRARIES} 
)

target_link_libraries(trace_rotation_node
  ${catkin_LIBRARIES}
)

## Mark executables and/or libraries for installation
install(TARGETS motor_controller_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
