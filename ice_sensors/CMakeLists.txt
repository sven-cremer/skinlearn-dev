cmake_minimum_required(VERSION 2.8.3)
project(ice_sensors)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  ice_msgs
  roscpp
  std_msgs
  visualization_msgs
  pr2_gripper_sensor_controller
  serial
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)


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
#  LIBRARIES ice_sensors
#  CATKIN_DEPENDS cmake_modules geometry_msgs ice_msgs roscpp std_msgs visualization_msgs
#  DEPENDS system_lib
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
  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(        bluetac_serial   src/bluetac_serial.cpp)
target_link_libraries( bluetac_serial   ${catkin_LIBRARIES} ${Boost_LIBRARIES})
target_link_libraries( bluetac_serial   pr2_gripper_sensor_controller_lib)
add_dependencies(      bluetac_serial   ice_msgs_generate_messages_cpp)


