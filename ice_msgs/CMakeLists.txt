cmake_minimum_required(VERSION 2.8.3)
project(ice_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  std_srvs
)

add_message_files(DIRECTORY msg FILES
  controllerFullData.msg
  controllerParam.msg
)

add_service_files(DIRECTORY srv FILES
  controllerParamUpdate.srv
  fixedWeightToggle.srv
  saveControllerData.srv
  setCartPose.srv
  setCircleRate.srv
)

generate_messages(DEPENDENCIES
  geometry_msgs
  std_msgs
  std_srvs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs
  std_srvs
)


