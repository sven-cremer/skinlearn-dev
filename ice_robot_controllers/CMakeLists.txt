cmake_minimum_required(VERSION 2.8.3)
project(ice_robot_controllers)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  control_toolbox
  cmake_modules
  ice_msgs
  kdl_conversions
  pluginlib
  pr2_controller_interface
  pr2_gripper_sensor_controller
  pr2_hardware_interface
  pr2_mechanism_model
  pr2_mechanism_msgs
  pr2_msgs
  realtime_tools
  roscpp
  std_srvs
  tf
  angles
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
# find_package(Boost REQUIRED COMPONENTS signals)

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
#  INCLUDE_DIRS include
#  LIBRARIES ice_robot_controllers
#  CATKIN_DEPENDS control_toolbox cmake_modules ice_msgs kdl_conversions pluginlib pr2_controller_interface pr2_gripper_sensor_controller pr2_hardware_interface pr2_mechanism_model pr2_mechanism_msgs pr2_msgs realtime_tools roscpp std_srvs tf angles
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(oelLIB
  src/least_squares.cpp
  src/kalman_estimate.cpp
)
add_library(ice_robot_controllers
  src/cartneuroController.cpp
)

#add_library(outer_loop src/outer_loop.cpp)
#target_link_libraries(outer_loop ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(ice_robot_controllers oelLIB ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ice_robot_controllers ice_msgs_generate_messages_cpp)
add_dependencies(ice_robot_controllers pr2_gripper_sensor_msgs_generate_messages_cpp)

add_dependencies(ice_robot_controllers ${ice_robot_controllers_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
pr2_enable_rpath(ice_robot_controllers)

install(TARGETS ice_robot_controllers
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES controller_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

## Declare a cpp executable
# add_executable(ice_robot_controllers_node src/ice_robot_controllers_node.cpp)

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(ice_robot_controllers_node ice_robot_controllers_generate_messages_cpp)

## Specify libraries to link a library or executable target against
# target_link_libraries(ice_robot_controllers_node
#   ${catkin_LIBRARIES}
# )

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ice_robot_controllers ice_robot_controllers_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
