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
add_library(digitalFilterLIB
  src/digitalFilter.cpp
)
add_library(ice_robot_controllers
  #src/cartneuroController.cpp
  src/adaptNeuroController.cpp
)

#add_library(outer_loop src/outer_loop.cpp)
#target_link_libraries(outer_loop ${catkin_LIBRARIES} ${Boost_LIBRARIES})

target_link_libraries(ice_robot_controllers
  oelLIB digitalFilterLIB nn_controller jt_controller nn_estimator
  ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(ice_robot_controllers ice_msgs_generate_messages_cpp)
add_dependencies(ice_robot_controllers pr2_gripper_sensor_msgs_generate_messages_cpp)

add_dependencies(ice_robot_controllers ${ice_robot_controllers_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
pr2_enable_rpath(ice_robot_controllers)

add_library(nn_controller include/csl/nn_controller.cpp)
target_link_libraries(nn_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(nn_estimator include/csl/nn_estimator.cpp)
target_link_libraries(nn_estimator ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_library(jt_controller include/ice_robot_controllers/jt_cartesian_controller.cpp)
target_link_libraries(jt_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(        test_digitalFilter   src/test_digitalFilter.cpp)
target_link_libraries( test_digitalFilter   ${catkin_LIBRARIES} ${Boost_LIBRARIES} digitalFilterLIB)

#############
## Install ##
#############

install(TARGETS ice_robot_controllers
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(FILES controller_plugins.xml
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

