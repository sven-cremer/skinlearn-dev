# Time
float64 dt

# ARMA filter
std_msgs/Float64MultiArray filterWeights
float64 filterNorm

# Neural network
# twoLayerNN net
float64 Vnorm
float64 Wnorm

# Actual trajectory
float64 x_x
float64 x_y
float64 x_z
float64 x_phi
float64 x_theta
float64 x_psi

float64 xdot_x
float64 xdot_y
float64 xdot_z
float64 xdot_phi
float64 xdot_theta
float64 xdot_psi

# Desired trajectory / human intent
float64 xdes_x
float64 xdes_y
float64 xdes_z
float64 xdes_phi
float64 xdes_theta
float64 xdes_psi
