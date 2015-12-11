cmake_minimum_required(VERSION 2.8.3)
project(pr2_cart)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  kdl_conversions
  pluginlib
  pr2_controller_interface
  pr2_controllers_msgs
  pr2_hardware_interface
  pr2_mechanism_model
  pr2_mechanism_msgs
  realtime_tools
  roscpp
  std_srvs
  ice_msgs
  apc_robot
)

## System dependencies are found with CMake's conventions
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
#  LIBRARIES pr2_cart
#  CATKIN_DEPENDS cmake_modules kdl_conversions pluginlib pr2_controller_interface pr2_controllers_msgs pr2_hardware_interface pr2_mechanism_model pr2_mechanism_msgs realtime_tools roscpp std_srvs ice_msgs apc_robot
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
add_library(           pr2_cart_lib   src/pr2_cart.cpp)
target_link_libraries( pr2_cart_lib   ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(      pr2_cart_lib   ice_msgs_generate_messages_cpp)

add_library(           pr2_cart_manager_lib  src/pr2_cart_manager.cpp )
target_link_libraries( pr2_cart_manager_lib  pr2_cart_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(      pr2_cart_manager_lib  ice_msgs_generate_messages_cpp)

## Declare a cpp executable
#add_executable(        pr2_cart_manager      src/pr2_cart_manager.cpp)
#target_link_libraries( pr2_cart_manager      pr2_cart_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
#add_dependencies(      pr2_cart_manager      pr2_cart_manager_generate_messages_cpp)
 
add_executable(         ft_sim             src/ft_simulation.cpp)     
target_link_libraries(  ft_sim             ${catkin_LIBRARIES} ${Boost_LIBRARIES})

# Keyboard interface
add_executable(        keyboard   src/keyboard.cpp)
target_link_libraries( keyboard   pr2_cart_manager_lib ${catkin_LIBRARIES} ${Boost_LIBRARIES})
add_dependencies(      keyboard   ice_msgs_generate_messages_cpp)


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
# install(TARGETS pr2_cart pr2_cart_node
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
