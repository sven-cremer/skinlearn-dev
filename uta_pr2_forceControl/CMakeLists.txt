cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#uncomment if you have defined messages
rosbuild_genmsg()
#uncomment if you have defined services
rosbuild_gensrv()

#common commands for building c++ executables and libraries

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

rosbuild_add_library( ${PROJECT_NAME} src/SystemModel.cpp )

rosbuild_add_library( cslLIB src/neural_network.cpp )
rosbuild_add_library( oelLIB src/least_squares.cpp src/kalman_estimate.cpp )
rosbuild_add_library( rbdlUtilLIB src/rbdl_urdfreader.cc )

rosbuild_add_library(pr2_controller_lib src/neuroadptController.cpp
										src/cartesianController.cpp 
                                        src/cartneuroController.cpp
                                        src/cartresnnController.cpp )
								       #src/explforceController.cpp
								       #src/impedanceController.cpp
								       #src/explforceController.cpp
 
target_link_libraries(pr2_controller_lib cslLIB )
target_link_libraries(pr2_controller_lib oelLIB )

#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)

rosbuild_add_executable(rbdlTest src/test/rbdlTest.cpp)
target_link_libraries(rbdlTest rbdlUtilLIB )

#rosbuild_add_executable(odeTest src/odeTest.cpp)
#rosbuild_add_executable(SystemModel_test src/test/SystemModel_test.cpp)
#target_link_libraries( SystemModel_test	oelLIB )

rosbuild_add_executable(controllerExperimenter src/controllerExperimenter.cpp)
target_link_libraries(controllerExperimenter yaml-cpp )

rosbuild_add_executable(SystemModel_test src/test/SystemModel_test.cpp)
target_link_libraries( SystemModel_test	oelLIB )

rosbuild_add_executable( pr2_joint_listener src/pr2_joint_listener.cpp )
rosbuild_add_executable( position_rarm src/position_rarm.cpp )

rosbuild_add_executable( interaction_force_estimator src/interaction_force_estimator.cpp )
target_link_libraries( interaction_force_estimator	oelLIB )

#target_link_libraries(example ${PROJECT_NAME})
