# Time
float64  dt

# Raw Force Data
float64 force_x
float64 force_y
float64 force_z
float64 torque_x
float64 torque_y
float64 torque_z

# Input Force Data
float64 r_eff_x
float64 r_eff_y
float64 r_eff_z
float64 r_trq_x
float64 r_trq_y
float64 r_trq_z

# Input reference efforts(torques)
float64 reference_eff_j0
float64 reference_eff_j1
float64 reference_eff_j2
float64 reference_eff_j3
float64 reference_eff_j4
float64 reference_eff_j5
float64 reference_eff_j6

# Model States
float64 m_cartPos_x 
float64 m_cartPos_y 
float64 m_cartPos_z 
float64 m_cartPos_Qx
float64 m_cartPos_Qy
float64 m_cartPos_Qz
float64 m_cartPos_QW

float64 m_pos_j0
float64 m_pos_j1
float64 m_pos_j2
float64 m_pos_j3
float64 m_pos_j4
float64 m_pos_j5
float64 m_pos_j6

float64 m_vel_j0
float64 m_vel_j1
float64 m_vel_j2
float64 m_vel_j3
float64 m_vel_j4
float64 m_vel_j5
float64 m_vel_j6

float64 m_acc_j0
float64 m_acc_j1
float64 m_acc_j2
float64 m_acc_j3
float64 m_acc_j4
float64 m_acc_j5
float64 m_acc_j6

float64 m_eff_j0
float64 m_eff_j1
float64 m_eff_j2
float64 m_eff_j3
float64 m_eff_j4
float64 m_eff_j5
float64 m_eff_j6

# Control Output
float64 control_eff_j0
float64 control_eff_j1
float64 control_eff_j2
float64 control_eff_j3
float64 control_eff_j4
float64 control_eff_j5
float64 control_eff_j6

# Robot States
float64 r_cartPos_x 
float64 r_cartPos_y 
float64 r_cartPos_z 
float64 r_cartPos_Qx
float64 r_cartPos_Qy
float64 r_cartPos_Qz
float64 r_cartPos_QW

float64 r_pos_j0
float64 r_pos_j1
float64 r_pos_j2
float64 r_pos_j3
float64 r_pos_j4
float64 r_pos_j5
float64 r_pos_j6

float64 r_vel_j0
float64 r_vel_j1
float64 r_vel_j2
float64 r_vel_j3
float64 r_vel_j4
float64 r_vel_j5
float64 r_vel_j6
	
float64 r_acc_j0
float64 r_acc_j1
float64 r_acc_j2
float64 r_acc_j3
float64 r_acc_j4
float64 r_acc_j5
float64 r_acc_j6

float64 r_eff_j0
float64 r_eff_j1
float64 r_eff_j2
float64 r_eff_j3
float64 r_eff_j4
float64 r_eff_j5
float64 r_eff_j6

# NN Params
float64 kappa
float64 Kv
float64 lambda
float64 Kz
float64 Zb
float64 F
float64 G
int64 inParams
int64 outParams
int64 hiddenNodes
int64 errorParams
float64 feedForwardForce
float64 nn_ON

# Model Params
float64 m
float64 d
float64 k
