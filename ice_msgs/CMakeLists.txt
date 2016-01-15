cmake_minimum_required(VERSION 2.8.3)
project(ice_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  std_srvs
)

add_message_files(DIRECTORY msg FILES
  # ice_robot_controllers
  neuroControllerState.msg
  controllerFullData.msg
  controllerParam.msg
  # pr2_cart
  combinedError.msg
  combinedTwistError.msg
  controllerState.msg
)

add_service_files(DIRECTORY srv FILES
  # ice_robot_controllers
  controllerParamUpdate.srv
  fixedWeightToggle.srv
  saveControllerData.srv
  setCartPose.srv
  setCircleRate.srv
  # pr2_cart
  getState.srv
  setCarteDofMatrix.srv
  setGains.srv
  setValue.srv
  setBool.srv
  setNNweights.srv
  getNNweights.srv
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


