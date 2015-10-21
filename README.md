Intelligent Control and Estimation (ICE) Library.
===

ROS packages being developed for the Intelligent Control and Estimation Library.  


Versions:

  * master (Groovy, rosbuild) - deprecated
  * catkin-devel (Hydro, catkin) - work in progress

# Install Dependencies
The following needs to be installed:

    sudo apt-get install ros-hydro-pr2-desktop
    sudo apt-get install ros-hydro-pr2-gripper-sensor*

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
    git clone git clone https://<USER>@bitbucket.org/<USER>/uta_pr2-dev2.git
    cd ..
    catkin_make

# Troubleshooting
If the build fails due to the pr2 gripper sensor package, try compiling it from source:

    cd ~/ice_ws/src
    git clone https://github.com/PR2/pr2_gripper_sensor.git -b hydro-devel


# Running

    roslaunch pr2_gazebo pr2_empty_world.launch
    roslaunch ice_robot_controllers cartneuroController.launch
    rosrun pr2_controller_manager pr2_controller_manager list
