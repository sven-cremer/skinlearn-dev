cmake_minimum_required(VERSION 2.8.3)
project(ice_experimenter)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  ice_msgs
  rosbag
  roscpp
  apc_robot
  sound_play
  trajectory_generator
  data_recorder
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(Boost REQUIRED COMPONENTS thread)

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
#  LIBRARIES ice_experimenter
#  CATKIN_DEPENDS cmake_modules ice_msgs rosbag roscpp apc_robot sound_play data_recorder
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
# Keyboard interface
add_executable(        ice_experimenter   src/keyboard.cpp)
target_link_libraries( ice_experimenter   ${catkin_LIBRARIES} ${Boost_LIBRARIES} yaml-cpp)
add_dependencies(      ice_experimenter   ice_msgs_generate_messages_cpp)
