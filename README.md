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

