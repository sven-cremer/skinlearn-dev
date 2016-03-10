# Time
float64 dt
float64 outer_dt

# Neural network
float64 RBFnorm
float64 TANHnorm

# Actual trajectory
JointVec q
JointVec qd

# Task trajectory
JointVec q_d
JointVec qd_d
JointVec qdd_d

# Torques
JointVec tau
JointVec tau_sat
