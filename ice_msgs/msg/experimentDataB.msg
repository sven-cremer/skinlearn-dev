# Time
float64 dt

# Neural network
# twoLayerNN net
float64 Vnorm
float64 Wnorm

# ARMA filter
#std_msgs/Float64MultiArray filterWeightsX
#std_msgs/Float64MultiArray filterWeightsY
float64 filterNormX
float64 filterNormY

# Force input
geometry_msgs/Vector3 tactile_force

# Actual trajectory
CartVec x
CartVec xd

# Human intent
CartVec x_i

# Task trajectory
CartVec x_d

# Model trajectory
CartVec x_m
CartVec xd_m
CartVec xdd_m