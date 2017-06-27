Intelligent Control and Estimation (ICE) Library.
===

ROS packages being developed for the Intelligent Control and Estimation Library.  


Branches:

  * master (Groovy, rosbuild) - deprecated
  * rbfnn (test with RBF activation functions) - incomplete
  * **catkin-devel** - started converting packages from rosmake to catkin
  * *indigo-devel* - main branch, work in progress

Working packages:

  * *ice_msgs*
  * *ice_robot_controllers*: contains the `adaptNeuroController` class
  * *ice_experimenter*: keyboard interface for running experiments
  * *ice_sensors*: code for tactile sensors
  * *pr2_cart*
  * *uta_pr2_...*: older packages that no longer work (and to be removed)

# Workspace setup
Use the following steps to setup a ROS indigo workspace:

    source /opt/ros/indigo/setup.bash
    mkdir ~/ice_ws/src -p
    cd ~/ice_ws/src
    catkin_init_workspace
    cd ..
    catkin_make

    echo "source ~/ice_ws/devel/setup.bash" >> ~/.bashrc
    source ~/.bashrc

# Install Dependencies
The following needs to be installed:

    sudo apt-get install ros-indigo-pr2-desktop
    sudo apt-get install ros-indigo-pr2-gripper-sensor*
    sudo apt-get install ros-indigo-cmake-modules

The *ice_sensors* package requires the Serial Communication Library:  

    cd ~/ice_ws/src
    git clone https://github.com/wjwwood/serial.git
    cd ..
    catkin_make

# Install ICE Library
Download the repositories and build:  

    cd ~/ice_ws/src
    git clone https://<USER>@bitbucket.org/<USER>/uta_pr2-dev2.git -b catkin-devel
    git clone https://<USER>@bitbucket.org/nextgensystems/robot_utilities.git
    cd ..
    catkin_make

To import the workspace into Eclipse IDE, do this once:

    cd ~/ice_ws
    catkin_make --force-cmake -G"Eclipse CDT4 - Unix Makefiles"

The branch can be changed using the following command:  

    cd /path/to/repository
    git checkout catkin-devel
    git status

# Troubleshooting
If the build fails due to the pr2 gripper sensor package, try compiling it from source:

    cd ~/ice_ws/src
    git clone https://github.com/PR2/pr2_gripper_sensor.git -b hydro-devel

# Demo 1: Tactile Sensors
Make sure the sensor data can be read from the USB port:
```
sudo apt-get install gtkterm
gtkterm
```
and do a ```shift+ctrl+s```. Choose the correct port parameters, i.e.
```
port: /dev/ttyUSB0
baud: 115200
```
If data is being published, close *gtkterm* (note: only one program can read from the port). Adjust the parameters in */ice_sensors/config/tactileBox.yaml* and start the ROS driver/RVIZ using
```
roslaunch ice_sensors tactileBox.launch
roslaunch ice_sensors rviz.launch
```
# Demo 2: Neuroadaptive controller
The most up-to-date controller is inside the *adaptNeuroController* class which can be launched with
```
roslaunch ice_robot_controllers adaptNeuroController.launch
```
To use the ICE experimenter:
```
roslaunch ice_experimenter ice_experimenter.launch NN:=true JT:=false
rosrun ice_experimenter ice_experimenter
```
Check the launch file and configuration files in `ice_experimenter/cfg` for more settings.
*Note*: Run the keyboard interface on the PR2 to avoid timing offsets during data collection.

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
roslaunch ice_robot_controllers adaptNeuroController.launch
```
Manually start the controller with the *pr2_controller_manager*
```
rosrun pr2_controller_manager pr2_controller_manager list
rosrun pr2_controller_manager pr2_controller_manager stop l_arm_controller
rosrun pr2_controller_manager pr2_controller_manager start pr2_adaptNeuroController
```
or use the keyboard utility:
```
rosrun ice_experimenter ice_experimenter
```
To manually apply a force:  
```
rosservice call /gazebo/apply_body_wrench "body_name: 'l_gripper_l_finger_tip_link'
reference_frame: 'l_gripper_l_finger_tip_link'
reference_point: {x: 0.0, y: 0.0, z: 0.0}
wrench:
  force: {x: 1.0, y: 4.0, z: 0.0}
  torque: {x: 0.0, y: 0.0, z: 0.0}
start_time: {secs: 0, nsecs: 0}
duration: {secs: 10, nsecs: 0}" 
```
*Note*: The simulated robot behaves differently from the real PR2. Maybe the F/T sensors change the kinematics and have to be added to the Gazebo model.

# Collecting data
Starting Experiment A after modifying *pr2_controller.yaml*:
```
roslaunch ice_robot_controllers adaptNeuroController.launch
rosservice call /pr2_adaptNeuroController/runExperimentA "value: 0.5"
```
Saving data:
```
rostopic echo -p /pr2_adaptNeuroController/experimentDataA > datafile.rtp
rosservice call /pr2_adaptNeuroController/publishExpData "{}"
```
In  future, the *ice_experimenter* will automatically save the data in a rosbag.

# Topics
```
rostopic list | grep pr2_adapt
rostopic echo /pr2_adaptNeuroController/state/x -c
rostopic echo /pr2_adaptNeuroController/state/x_desi -c
```
