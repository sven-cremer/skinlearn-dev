PR2 cart
===

Realtime impedance and velocity controller for PR2 cart following behavior.  

# Running in Simulation
Start gazebo:  
```
    roslaunch pr2_gazebo pr2_empty_world.launch
```
Spawn the controller and start it:
```
    roslaunch pr2_cart pr2_cart.launch
    rosrun pr2_controller_manager pr2_controller_manager list
    rosrun pr2_controller_manager pr2_controller_manager stop r_gripper_controller
    rosrun pr2_controller_manager pr2_controller_manager stop l_gripper_controller
    rosrun pr2_controller_manager pr2_controller_manager start pr2_cart
```
or simply launch:
```
    roslaunch pr2_cart start.launch
```

Apply some forces:  
```
    rosrun pr2_cart ft_sim
```
