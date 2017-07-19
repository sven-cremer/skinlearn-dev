Header header
float64 dt
int64 missed_updates_counter
int64 invalid_value_counter

## Robot state
geometry_msgs/PoseStamped x
geometry_msgs/Twist xd
geometry_msgs/Wrench force_measured
float64[] q
#std_msgs/Float64MultiArray J
#std_msgs/Float64MultiArray N

## Reference trajectory
geometry_msgs/PoseStamped x_desi
geometry_msgs/PoseStamped x_desi_filtered
geometry_msgs/Twist xd_desi
geometry_msgs/Twist x_err

## Controller
geometry_msgs/Wrench force_c
geometry_msgs/Wrench force_h
float64[] tau_posture
float64[] tau_c

## NN parameters
float64 W_norm
float64 V_norm
#std_msgs/Float64MultiArray W
#std_msgs/Float64MultiArray V
float64[] Gamma
float64[] Lambda

## Estimator
geometry_msgs/Pose x_hat
geometry_msgs/Pose xd_hat
float64[] Kh
float64[] Dh
float64 U_norm_traj
float64 V_norm_gain
float64[] nne_ea
float64[] nne_s

## ARMA weights
float64[] armaX
float64[] armaY
