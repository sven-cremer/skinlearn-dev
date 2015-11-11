PR2 cart
===

Realtime impedance and velocity controller for PR2 cart following behavior.  

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
