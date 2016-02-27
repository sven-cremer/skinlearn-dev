# Time
float64 dt
float64 outer_dt

# Neural network
# twoLayerNN net
float64 Vnorm
float64 Wnorm

# ARMA filter
#std_msgs/Float64MultiArray filterWeightsX
#std_msgs/Float64MultiArray filterWeightsY
filterWeights flexi0
filterWeights flexi1
filterWeights flexi2
filterWeights flexi3
float64 filterNormX
float64 filterNormY

# Force input
#geometry_msgs/Vector3 tactile_force
float64 tactile_data_0
float64 tactile_data_1
float64 tactile_data_2
float64 tactile_data_3

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
