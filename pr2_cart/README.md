PR2 cart
===

Realtime impedance and velocity controller for PR2 cart following behavior. 
 
# Running on the PR2
First, you need to set up the PR2 to run in hydro. Normally, 1st generation PR2's don't have ROS hydro in their default configuration. UTARI's PR2 has been modified to run on both ROS fuerte and ROS hydro. Bofore running ```robot start ```, run the following commands 
```
robot hydro
source ~/ice_ws/devel/setup.bash
robot start
```
then following the following steps:
   1. Initialize PR2 cart manager on the PR2
   ```
   roslaunch pr2_cart pr2_cart.launch
   ```
   2. Initialize PR2 keyboard on a separate computer terminal
   ```
   rosrun pr2_cart keyboard
   ```


# Running in Simulation
Start gazebo:  
```
roslaunch pr2_gazebo pr2_empty_world.launch
```
Spawn the controller and start keyboard interface:
```
roslaunch pr2_cart pr2_cart.launch
rosrun pr2_cart keyboard
```
or simply launch:
```
roslaunch pr2_cart start.launch
rosrun pr2_controller_manager pr2_controller_manager list
```

Apply some forces:  
```
rosrun pr2_cart ft_sim
```
