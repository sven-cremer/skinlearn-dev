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

# Cartesian task reference                
float64 taskRef_x
float64 taskRef_y
float64 taskRef_z
float64 taskRef_phi
float64 taskRef_theta
float64 taskRef_psi

# Cartesian task reference                
float64 taskRefModel_x
float64 taskRefModel_y
float64 taskRefModel_z
float64 taskRefModel_phi
float64 taskRefModel_theta
float64 taskRefModel_psi

# Model States
float64 m_cartPos_x 
float64 m_cartPos_y 
float64 m_cartPos_z 
float64 m_cartPos_Qx
float64 m_cartPos_Qy
float64 m_cartPos_Qz
float64 m_cartPos_QW

float64 m_pos_x
float64 m_pos_y
float64 m_pos_z
       
float64 m_vel_x
float64 m_vel_y
float64 m_vel_z
       
float64 m_acc_x
float64 m_acc_y
float64 m_acc_z

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

# Joint lower limit 
float64 l_limit_0
float64 l_limit_1
float64 l_limit_2
float64 l_limit_3
float64 l_limit_4
float64 l_limit_5
float64 l_limit_6
         
# Joint upper limit
float64 u_limit_0
float64 u_limit_1
float64 u_limit_2
float64 u_limit_3
float64 u_limit_4
float64 u_limit_5
float64 u_limit_6

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

# Cart params
float64 cartPos_Kp_x
float64 cartPos_Kp_y
float64 cartPos_Kp_z
float64 cartPos_Kd_x
float64 cartPos_Kd_y
float64 cartPos_Kd_z

float64 cartRot_Kp_x
float64 cartRot_Kp_y
float64 cartRot_Kp_z
float64 cartRot_Kd_x
float64 cartRot_Kd_y
float64 cartRot_Kd_z

bool useCurrentCartPose
bool useNullspacePose

float64 cartIniX    
float64 cartIniY    
float64 cartIniZ
float64 cartIniRoll 
float64 cartIniPitch
float64 cartIniYaw  
        
float64 cartDesX    
float64 cartDesY    
float64 cartDesZ    
float64 cartDesRoll 
float64 cartDesPitch
float64 cartDesYaw  

# Model Params
float64 m
float64 d
float64 k
