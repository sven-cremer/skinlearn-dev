Intelligent Control and Estimation (ICE) Library.
===

ROS packages being developed for the Intelligent Control and Estimation Library.  


Versions:

  * master (Groovy, rosbuild) - deprecated
  * catkin-devel (Hydro, catkin) - work in progress

Ported packages:

  * ice_msgs
  * ice_robot_controllers
  * pr2_cart


# Install Dependencies
The following needs to be installed:

    sudo apt-get install ros-hydro-pr2-desktop
    sudo apt-get install ros-hydro-pr2-gripper-sensor*
    sudo apt-get install ros-hydro-cmake-modules

# Install Library
Workspace setup:  

    source /opt/ros/hydro/setup.bash
    mkdir ~/ice_ws/src -p
    cd ~/ice_ws/src
    catkin_init_workspace
    cd ..
    catkin_make

    echo "source ~/ice_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

Download repository and build:  

    cd ~/ice_ws/src
    git clone https://<USER>@bitbucket.org/<USER>/uta_pr2-dev2.git -b catkin-devel
    cd ..
    catkin_make

The branch can also be changed using the following command:  

    cd /path/to/repository
    git checkout catkin-devel
    git status

# Troubleshooting
If the build fails due to the pr2 gripper sensor package, try compiling it from source:

    cd ~/ice_ws/src
    git clone https://github.com/PR2/pr2_gripper_sensor.git -b hydro-devel


# Running in Simulation
Add the following settings to bashrc:  
```
export ROBOT=sim
export ROS_MASTER_URI=http://localhost:11311
```
Make sure the following parameters in *ice_robot_controllers/config/pr2_controller.yaml* are set to false:  
```
useFTinput         : false
forceTorqueOn      : false
useFlexiForce      : false
accelerometerOn    : false
```
Start gazebo and launch the controller:  
```
roslaunch pr2_gazebo pr2_empty_world.launch
roslaunch ice_robot_controllers cartneuroController.launch
rosrun pr2_controller_manager pr2_controller_manager list
```
Apply a force:  
```
rosservice call /gazebo/apply_body_wrench "body_name: 'l_gripper_l_finger_tip_link'
reference_frame: 'l_gripper_l_finger_tip_link'
reference_point: {x: 0.0, y: 0.0, z: 0.0}
wrench:
  force: {x: 1.0, y: 4.0, z: 0.0}
  torque: {x: 0.0, y: 0.0, z: 0.0}
start_time: {secs: 0, nsecs: 0}
duration: {secs: 60, nsecs: 0}" 
```
