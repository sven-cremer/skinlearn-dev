"""autogenerated by genpy from neuroadaptive_msgs/controllerFullData.msg. Do not edit."""
import sys
python3 = True if sys.hexversion > 0x03000000 else False
import genpy
import struct


class controllerFullData(genpy.Message):
  _md5sum = "3b700d9b967b330bbafaa74bfb7ba442"
  _type = "neuroadaptive_msgs/controllerFullData"
  _has_header = False #flag to mark the presence of a Header object
  _full_text = """# Time
float64  dt

# Force Data
float64 force_x
float64 force_y
float64 force_z
float64 torque_x
float64 torque_y
float64 torque_z

# Accelerometer data
float64 acc_x
float64 acc_y
float64 acc_z

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

# Ref Model Params
float64 m
float64 d
float64 k

# Task Ref Params
float64 task_mA
float64 task_mB

# Use fixed weights
float64 fixedFilterWeights

# Filter Weights
float64 w0
float64 w1
float64 w2
float64 w3
float64 w4
float64 w5
float64 w6
float64 w7
"""
  __slots__ = ['dt','force_x','force_y','force_z','torque_x','torque_y','torque_z','acc_x','acc_y','acc_z','r_eff_x','r_eff_y','r_eff_z','r_trq_x','r_trq_y','r_trq_z','reference_eff_j0','reference_eff_j1','reference_eff_j2','reference_eff_j3','reference_eff_j4','reference_eff_j5','reference_eff_j6','taskRef_x','taskRef_y','taskRef_z','taskRef_phi','taskRef_theta','taskRef_psi','taskRefModel_x','taskRefModel_y','taskRefModel_z','taskRefModel_phi','taskRefModel_theta','taskRefModel_psi','m_cartPos_x','m_cartPos_y','m_cartPos_z','m_cartPos_Qx','m_cartPos_Qy','m_cartPos_Qz','m_cartPos_QW','m_pos_x','m_pos_y','m_pos_z','m_vel_x','m_vel_y','m_vel_z','m_acc_x','m_acc_y','m_acc_z','m_pos_j0','m_pos_j1','m_pos_j2','m_pos_j3','m_pos_j4','m_pos_j5','m_pos_j6','m_vel_j0','m_vel_j1','m_vel_j2','m_vel_j3','m_vel_j4','m_vel_j5','m_vel_j6','m_acc_j0','m_acc_j1','m_acc_j2','m_acc_j3','m_acc_j4','m_acc_j5','m_acc_j6','m_eff_j0','m_eff_j1','m_eff_j2','m_eff_j3','m_eff_j4','m_eff_j5','m_eff_j6','control_eff_j0','control_eff_j1','control_eff_j2','control_eff_j3','control_eff_j4','control_eff_j5','control_eff_j6','r_cartPos_x','r_cartPos_y','r_cartPos_z','r_cartPos_Qx','r_cartPos_Qy','r_cartPos_Qz','r_cartPos_QW','r_pos_j0','r_pos_j1','r_pos_j2','r_pos_j3','r_pos_j4','r_pos_j5','r_pos_j6','r_vel_j0','r_vel_j1','r_vel_j2','r_vel_j3','r_vel_j4','r_vel_j5','r_vel_j6','r_acc_j0','r_acc_j1','r_acc_j2','r_acc_j3','r_acc_j4','r_acc_j5','r_acc_j6','r_eff_j0','r_eff_j1','r_eff_j2','r_eff_j3','r_eff_j4','r_eff_j5','r_eff_j6','l_limit_0','l_limit_1','l_limit_2','l_limit_3','l_limit_4','l_limit_5','l_limit_6','u_limit_0','u_limit_1','u_limit_2','u_limit_3','u_limit_4','u_limit_5','u_limit_6','kappa','Kv','lambda_','Kz','Zb','F','G','inParams','outParams','hiddenNodes','errorParams','feedForwardForce','nn_ON','cartPos_Kp_x','cartPos_Kp_y','cartPos_Kp_z','cartPos_Kd_x','cartPos_Kd_y','cartPos_Kd_z','cartRot_Kp_x','cartRot_Kp_y','cartRot_Kp_z','cartRot_Kd_x','cartRot_Kd_y','cartRot_Kd_z','useCurrentCartPose','useNullspacePose','cartIniX','cartIniY','cartIniZ','cartIniRoll','cartIniPitch','cartIniYaw','cartDesX','cartDesY','cartDesZ','cartDesRoll','cartDesPitch','cartDesYaw','m','d','k','task_mA','task_mB','fixedFilterWeights','w0','w1','w2','w3','w4','w5','w6','w7']
  _slot_types = ['float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','int64','int64','int64','int64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','bool','bool','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64','float64']

  def __init__(self, *args, **kwds):
    """
    Constructor. Any message fields that are implicitly/explicitly
    set to None will be assigned a default value. The recommend
    use is keyword arguments as this is more robust to future message
    changes.  You cannot mix in-order arguments and keyword arguments.

    The available fields are:
       dt,force_x,force_y,force_z,torque_x,torque_y,torque_z,acc_x,acc_y,acc_z,r_eff_x,r_eff_y,r_eff_z,r_trq_x,r_trq_y,r_trq_z,reference_eff_j0,reference_eff_j1,reference_eff_j2,reference_eff_j3,reference_eff_j4,reference_eff_j5,reference_eff_j6,taskRef_x,taskRef_y,taskRef_z,taskRef_phi,taskRef_theta,taskRef_psi,taskRefModel_x,taskRefModel_y,taskRefModel_z,taskRefModel_phi,taskRefModel_theta,taskRefModel_psi,m_cartPos_x,m_cartPos_y,m_cartPos_z,m_cartPos_Qx,m_cartPos_Qy,m_cartPos_Qz,m_cartPos_QW,m_pos_x,m_pos_y,m_pos_z,m_vel_x,m_vel_y,m_vel_z,m_acc_x,m_acc_y,m_acc_z,m_pos_j0,m_pos_j1,m_pos_j2,m_pos_j3,m_pos_j4,m_pos_j5,m_pos_j6,m_vel_j0,m_vel_j1,m_vel_j2,m_vel_j3,m_vel_j4,m_vel_j5,m_vel_j6,m_acc_j0,m_acc_j1,m_acc_j2,m_acc_j3,m_acc_j4,m_acc_j5,m_acc_j6,m_eff_j0,m_eff_j1,m_eff_j2,m_eff_j3,m_eff_j4,m_eff_j5,m_eff_j6,control_eff_j0,control_eff_j1,control_eff_j2,control_eff_j3,control_eff_j4,control_eff_j5,control_eff_j6,r_cartPos_x,r_cartPos_y,r_cartPos_z,r_cartPos_Qx,r_cartPos_Qy,r_cartPos_Qz,r_cartPos_QW,r_pos_j0,r_pos_j1,r_pos_j2,r_pos_j3,r_pos_j4,r_pos_j5,r_pos_j6,r_vel_j0,r_vel_j1,r_vel_j2,r_vel_j3,r_vel_j4,r_vel_j5,r_vel_j6,r_acc_j0,r_acc_j1,r_acc_j2,r_acc_j3,r_acc_j4,r_acc_j5,r_acc_j6,r_eff_j0,r_eff_j1,r_eff_j2,r_eff_j3,r_eff_j4,r_eff_j5,r_eff_j6,l_limit_0,l_limit_1,l_limit_2,l_limit_3,l_limit_4,l_limit_5,l_limit_6,u_limit_0,u_limit_1,u_limit_2,u_limit_3,u_limit_4,u_limit_5,u_limit_6,kappa,Kv,lambda_,Kz,Zb,F,G,inParams,outParams,hiddenNodes,errorParams,feedForwardForce,nn_ON,cartPos_Kp_x,cartPos_Kp_y,cartPos_Kp_z,cartPos_Kd_x,cartPos_Kd_y,cartPos_Kd_z,cartRot_Kp_x,cartRot_Kp_y,cartRot_Kp_z,cartRot_Kd_x,cartRot_Kd_y,cartRot_Kd_z,useCurrentCartPose,useNullspacePose,cartIniX,cartIniY,cartIniZ,cartIniRoll,cartIniPitch,cartIniYaw,cartDesX,cartDesY,cartDesZ,cartDesRoll,cartDesPitch,cartDesYaw,m,d,k,task_mA,task_mB,fixedFilterWeights,w0,w1,w2,w3,w4,w5,w6,w7

    :param args: complete set of field values, in .msg order
    :param kwds: use keyword arguments corresponding to message field names
    to set specific fields.
    """
    if args or kwds:
      super(controllerFullData, self).__init__(*args, **kwds)
      #message fields cannot be None, assign default values for those that are
      if self.dt is None:
        self.dt = 0.
      if self.force_x is None:
        self.force_x = 0.
      if self.force_y is None:
        self.force_y = 0.
      if self.force_z is None:
        self.force_z = 0.
      if self.torque_x is None:
        self.torque_x = 0.
      if self.torque_y is None:
        self.torque_y = 0.
      if self.torque_z is None:
        self.torque_z = 0.
      if self.acc_x is None:
        self.acc_x = 0.
      if self.acc_y is None:
        self.acc_y = 0.
      if self.acc_z is None:
        self.acc_z = 0.
      if self.r_eff_x is None:
        self.r_eff_x = 0.
      if self.r_eff_y is None:
        self.r_eff_y = 0.
      if self.r_eff_z is None:
        self.r_eff_z = 0.
      if self.r_trq_x is None:
        self.r_trq_x = 0.
      if self.r_trq_y is None:
        self.r_trq_y = 0.
      if self.r_trq_z is None:
        self.r_trq_z = 0.
      if self.reference_eff_j0 is None:
        self.reference_eff_j0 = 0.
      if self.reference_eff_j1 is None:
        self.reference_eff_j1 = 0.
      if self.reference_eff_j2 is None:
        self.reference_eff_j2 = 0.
      if self.reference_eff_j3 is None:
        self.reference_eff_j3 = 0.
      if self.reference_eff_j4 is None:
        self.reference_eff_j4 = 0.
      if self.reference_eff_j5 is None:
        self.reference_eff_j5 = 0.
      if self.reference_eff_j6 is None:
        self.reference_eff_j6 = 0.
      if self.taskRef_x is None:
        self.taskRef_x = 0.
      if self.taskRef_y is None:
        self.taskRef_y = 0.
      if self.taskRef_z is None:
        self.taskRef_z = 0.
      if self.taskRef_phi is None:
        self.taskRef_phi = 0.
      if self.taskRef_theta is None:
        self.taskRef_theta = 0.
      if self.taskRef_psi is None:
        self.taskRef_psi = 0.
      if self.taskRefModel_x is None:
        self.taskRefModel_x = 0.
      if self.taskRefModel_y is None:
        self.taskRefModel_y = 0.
      if self.taskRefModel_z is None:
        self.taskRefModel_z = 0.
      if self.taskRefModel_phi is None:
        self.taskRefModel_phi = 0.
      if self.taskRefModel_theta is None:
        self.taskRefModel_theta = 0.
      if self.taskRefModel_psi is None:
        self.taskRefModel_psi = 0.
      if self.m_cartPos_x is None:
        self.m_cartPos_x = 0.
      if self.m_cartPos_y is None:
        self.m_cartPos_y = 0.
      if self.m_cartPos_z is None:
        self.m_cartPos_z = 0.
      if self.m_cartPos_Qx is None:
        self.m_cartPos_Qx = 0.
      if self.m_cartPos_Qy is None:
        self.m_cartPos_Qy = 0.
      if self.m_cartPos_Qz is None:
        self.m_cartPos_Qz = 0.
      if self.m_cartPos_QW is None:
        self.m_cartPos_QW = 0.
      if self.m_pos_x is None:
        self.m_pos_x = 0.
      if self.m_pos_y is None:
        self.m_pos_y = 0.
      if self.m_pos_z is None:
        self.m_pos_z = 0.
      if self.m_vel_x is None:
        self.m_vel_x = 0.
      if self.m_vel_y is None:
        self.m_vel_y = 0.
      if self.m_vel_z is None:
        self.m_vel_z = 0.
      if self.m_acc_x is None:
        self.m_acc_x = 0.
      if self.m_acc_y is None:
        self.m_acc_y = 0.
      if self.m_acc_z is None:
        self.m_acc_z = 0.
      if self.m_pos_j0 is None:
        self.m_pos_j0 = 0.
      if self.m_pos_j1 is None:
        self.m_pos_j1 = 0.
      if self.m_pos_j2 is None:
        self.m_pos_j2 = 0.
      if self.m_pos_j3 is None:
        self.m_pos_j3 = 0.
      if self.m_pos_j4 is None:
        self.m_pos_j4 = 0.
      if self.m_pos_j5 is None:
        self.m_pos_j5 = 0.
      if self.m_pos_j6 is None:
        self.m_pos_j6 = 0.
      if self.m_vel_j0 is None:
        self.m_vel_j0 = 0.
      if self.m_vel_j1 is None:
        self.m_vel_j1 = 0.
      if self.m_vel_j2 is None:
        self.m_vel_j2 = 0.
      if self.m_vel_j3 is None:
        self.m_vel_j3 = 0.
      if self.m_vel_j4 is None:
        self.m_vel_j4 = 0.
      if self.m_vel_j5 is None:
        self.m_vel_j5 = 0.
      if self.m_vel_j6 is None:
        self.m_vel_j6 = 0.
      if self.m_acc_j0 is None:
        self.m_acc_j0 = 0.
      if self.m_acc_j1 is None:
        self.m_acc_j1 = 0.
      if self.m_acc_j2 is None:
        self.m_acc_j2 = 0.
      if self.m_acc_j3 is None:
        self.m_acc_j3 = 0.
      if self.m_acc_j4 is None:
        self.m_acc_j4 = 0.
      if self.m_acc_j5 is None:
        self.m_acc_j5 = 0.
      if self.m_acc_j6 is None:
        self.m_acc_j6 = 0.
      if self.m_eff_j0 is None:
        self.m_eff_j0 = 0.
      if self.m_eff_j1 is None:
        self.m_eff_j1 = 0.
      if self.m_eff_j2 is None:
        self.m_eff_j2 = 0.
      if self.m_eff_j3 is None:
        self.m_eff_j3 = 0.
      if self.m_eff_j4 is None:
        self.m_eff_j4 = 0.
      if self.m_eff_j5 is None:
        self.m_eff_j5 = 0.
      if self.m_eff_j6 is None:
        self.m_eff_j6 = 0.
      if self.control_eff_j0 is None:
        self.control_eff_j0 = 0.
      if self.control_eff_j1 is None:
        self.control_eff_j1 = 0.
      if self.control_eff_j2 is None:
        self.control_eff_j2 = 0.
      if self.control_eff_j3 is None:
        self.control_eff_j3 = 0.
      if self.control_eff_j4 is None:
        self.control_eff_j4 = 0.
      if self.control_eff_j5 is None:
        self.control_eff_j5 = 0.
      if self.control_eff_j6 is None:
        self.control_eff_j6 = 0.
      if self.r_cartPos_x is None:
        self.r_cartPos_x = 0.
      if self.r_cartPos_y is None:
        self.r_cartPos_y = 0.
      if self.r_cartPos_z is None:
        self.r_cartPos_z = 0.
      if self.r_cartPos_Qx is None:
        self.r_cartPos_Qx = 0.
      if self.r_cartPos_Qy is None:
        self.r_cartPos_Qy = 0.
      if self.r_cartPos_Qz is None:
        self.r_cartPos_Qz = 0.
      if self.r_cartPos_QW is None:
        self.r_cartPos_QW = 0.
      if self.r_pos_j0 is None:
        self.r_pos_j0 = 0.
      if self.r_pos_j1 is None:
        self.r_pos_j1 = 0.
      if self.r_pos_j2 is None:
        self.r_pos_j2 = 0.
      if self.r_pos_j3 is None:
        self.r_pos_j3 = 0.
      if self.r_pos_j4 is None:
        self.r_pos_j4 = 0.
      if self.r_pos_j5 is None:
        self.r_pos_j5 = 0.
      if self.r_pos_j6 is None:
        self.r_pos_j6 = 0.
      if self.r_vel_j0 is None:
        self.r_vel_j0 = 0.
      if self.r_vel_j1 is None:
        self.r_vel_j1 = 0.
      if self.r_vel_j2 is None:
        self.r_vel_j2 = 0.
      if self.r_vel_j3 is None:
        self.r_vel_j3 = 0.
      if self.r_vel_j4 is None:
        self.r_vel_j4 = 0.
      if self.r_vel_j5 is None:
        self.r_vel_j5 = 0.
      if self.r_vel_j6 is None:
        self.r_vel_j6 = 0.
      if self.r_acc_j0 is None:
        self.r_acc_j0 = 0.
      if self.r_acc_j1 is None:
        self.r_acc_j1 = 0.
      if self.r_acc_j2 is None:
        self.r_acc_j2 = 0.
      if self.r_acc_j3 is None:
        self.r_acc_j3 = 0.
      if self.r_acc_j4 is None:
        self.r_acc_j4 = 0.
      if self.r_acc_j5 is None:
        self.r_acc_j5 = 0.
      if self.r_acc_j6 is None:
        self.r_acc_j6 = 0.
      if self.r_eff_j0 is None:
        self.r_eff_j0 = 0.
      if self.r_eff_j1 is None:
        self.r_eff_j1 = 0.
      if self.r_eff_j2 is None:
        self.r_eff_j2 = 0.
      if self.r_eff_j3 is None:
        self.r_eff_j3 = 0.
      if self.r_eff_j4 is None:
        self.r_eff_j4 = 0.
      if self.r_eff_j5 is None:
        self.r_eff_j5 = 0.
      if self.r_eff_j6 is None:
        self.r_eff_j6 = 0.
      if self.l_limit_0 is None:
        self.l_limit_0 = 0.
      if self.l_limit_1 is None:
        self.l_limit_1 = 0.
      if self.l_limit_2 is None:
        self.l_limit_2 = 0.
      if self.l_limit_3 is None:
        self.l_limit_3 = 0.
      if self.l_limit_4 is None:
        self.l_limit_4 = 0.
      if self.l_limit_5 is None:
        self.l_limit_5 = 0.
      if self.l_limit_6 is None:
        self.l_limit_6 = 0.
      if self.u_limit_0 is None:
        self.u_limit_0 = 0.
      if self.u_limit_1 is None:
        self.u_limit_1 = 0.
      if self.u_limit_2 is None:
        self.u_limit_2 = 0.
      if self.u_limit_3 is None:
        self.u_limit_3 = 0.
      if self.u_limit_4 is None:
        self.u_limit_4 = 0.
      if self.u_limit_5 is None:
        self.u_limit_5 = 0.
      if self.u_limit_6 is None:
        self.u_limit_6 = 0.
      if self.kappa is None:
        self.kappa = 0.
      if self.Kv is None:
        self.Kv = 0.
      if self.lambda_ is None:
        self.lambda_ = 0.
      if self.Kz is None:
        self.Kz = 0.
      if self.Zb is None:
        self.Zb = 0.
      if self.F is None:
        self.F = 0.
      if self.G is None:
        self.G = 0.
      if self.inParams is None:
        self.inParams = 0
      if self.outParams is None:
        self.outParams = 0
      if self.hiddenNodes is None:
        self.hiddenNodes = 0
      if self.errorParams is None:
        self.errorParams = 0
      if self.feedForwardForce is None:
        self.feedForwardForce = 0.
      if self.nn_ON is None:
        self.nn_ON = 0.
      if self.cartPos_Kp_x is None:
        self.cartPos_Kp_x = 0.
      if self.cartPos_Kp_y is None:
        self.cartPos_Kp_y = 0.
      if self.cartPos_Kp_z is None:
        self.cartPos_Kp_z = 0.
      if self.cartPos_Kd_x is None:
        self.cartPos_Kd_x = 0.
      if self.cartPos_Kd_y is None:
        self.cartPos_Kd_y = 0.
      if self.cartPos_Kd_z is None:
        self.cartPos_Kd_z = 0.
      if self.cartRot_Kp_x is None:
        self.cartRot_Kp_x = 0.
      if self.cartRot_Kp_y is None:
        self.cartRot_Kp_y = 0.
      if self.cartRot_Kp_z is None:
        self.cartRot_Kp_z = 0.
      if self.cartRot_Kd_x is None:
        self.cartRot_Kd_x = 0.
      if self.cartRot_Kd_y is None:
        self.cartRot_Kd_y = 0.
      if self.cartRot_Kd_z is None:
        self.cartRot_Kd_z = 0.
      if self.useCurrentCartPose is None:
        self.useCurrentCartPose = False
      if self.useNullspacePose is None:
        self.useNullspacePose = False
      if self.cartIniX is None:
        self.cartIniX = 0.
      if self.cartIniY is None:
        self.cartIniY = 0.
      if self.cartIniZ is None:
        self.cartIniZ = 0.
      if self.cartIniRoll is None:
        self.cartIniRoll = 0.
      if self.cartIniPitch is None:
        self.cartIniPitch = 0.
      if self.cartIniYaw is None:
        self.cartIniYaw = 0.
      if self.cartDesX is None:
        self.cartDesX = 0.
      if self.cartDesY is None:
        self.cartDesY = 0.
      if self.cartDesZ is None:
        self.cartDesZ = 0.
      if self.cartDesRoll is None:
        self.cartDesRoll = 0.
      if self.cartDesPitch is None:
        self.cartDesPitch = 0.
      if self.cartDesYaw is None:
        self.cartDesYaw = 0.
      if self.m is None:
        self.m = 0.
      if self.d is None:
        self.d = 0.
      if self.k is None:
        self.k = 0.
      if self.task_mA is None:
        self.task_mA = 0.
      if self.task_mB is None:
        self.task_mB = 0.
      if self.fixedFilterWeights is None:
        self.fixedFilterWeights = 0.
      if self.w0 is None:
        self.w0 = 0.
      if self.w1 is None:
        self.w1 = 0.
      if self.w2 is None:
        self.w2 = 0.
      if self.w3 is None:
        self.w3 = 0.
      if self.w4 is None:
        self.w4 = 0.
      if self.w5 is None:
        self.w5 = 0.
      if self.w6 is None:
        self.w6 = 0.
      if self.w7 is None:
        self.w7 = 0.
    else:
      self.dt = 0.
      self.force_x = 0.
      self.force_y = 0.
      self.force_z = 0.
      self.torque_x = 0.
      self.torque_y = 0.
      self.torque_z = 0.
      self.acc_x = 0.
      self.acc_y = 0.
      self.acc_z = 0.
      self.r_eff_x = 0.
      self.r_eff_y = 0.
      self.r_eff_z = 0.
      self.r_trq_x = 0.
      self.r_trq_y = 0.
      self.r_trq_z = 0.
      self.reference_eff_j0 = 0.
      self.reference_eff_j1 = 0.
      self.reference_eff_j2 = 0.
      self.reference_eff_j3 = 0.
      self.reference_eff_j4 = 0.
      self.reference_eff_j5 = 0.
      self.reference_eff_j6 = 0.
      self.taskRef_x = 0.
      self.taskRef_y = 0.
      self.taskRef_z = 0.
      self.taskRef_phi = 0.
      self.taskRef_theta = 0.
      self.taskRef_psi = 0.
      self.taskRefModel_x = 0.
      self.taskRefModel_y = 0.
      self.taskRefModel_z = 0.
      self.taskRefModel_phi = 0.
      self.taskRefModel_theta = 0.
      self.taskRefModel_psi = 0.
      self.m_cartPos_x = 0.
      self.m_cartPos_y = 0.
      self.m_cartPos_z = 0.
      self.m_cartPos_Qx = 0.
      self.m_cartPos_Qy = 0.
      self.m_cartPos_Qz = 0.
      self.m_cartPos_QW = 0.
      self.m_pos_x = 0.
      self.m_pos_y = 0.
      self.m_pos_z = 0.
      self.m_vel_x = 0.
      self.m_vel_y = 0.
      self.m_vel_z = 0.
      self.m_acc_x = 0.
      self.m_acc_y = 0.
      self.m_acc_z = 0.
      self.m_pos_j0 = 0.
      self.m_pos_j1 = 0.
      self.m_pos_j2 = 0.
      self.m_pos_j3 = 0.
      self.m_pos_j4 = 0.
      self.m_pos_j5 = 0.
      self.m_pos_j6 = 0.
      self.m_vel_j0 = 0.
      self.m_vel_j1 = 0.
      self.m_vel_j2 = 0.
      self.m_vel_j3 = 0.
      self.m_vel_j4 = 0.
      self.m_vel_j5 = 0.
      self.m_vel_j6 = 0.
      self.m_acc_j0 = 0.
      self.m_acc_j1 = 0.
      self.m_acc_j2 = 0.
      self.m_acc_j3 = 0.
      self.m_acc_j4 = 0.
      self.m_acc_j5 = 0.
      self.m_acc_j6 = 0.
      self.m_eff_j0 = 0.
      self.m_eff_j1 = 0.
      self.m_eff_j2 = 0.
      self.m_eff_j3 = 0.
      self.m_eff_j4 = 0.
      self.m_eff_j5 = 0.
      self.m_eff_j6 = 0.
      self.control_eff_j0 = 0.
      self.control_eff_j1 = 0.
      self.control_eff_j2 = 0.
      self.control_eff_j3 = 0.
      self.control_eff_j4 = 0.
      self.control_eff_j5 = 0.
      self.control_eff_j6 = 0.
      self.r_cartPos_x = 0.
      self.r_cartPos_y = 0.
      self.r_cartPos_z = 0.
      self.r_cartPos_Qx = 0.
      self.r_cartPos_Qy = 0.
      self.r_cartPos_Qz = 0.
      self.r_cartPos_QW = 0.
      self.r_pos_j0 = 0.
      self.r_pos_j1 = 0.
      self.r_pos_j2 = 0.
      self.r_pos_j3 = 0.
      self.r_pos_j4 = 0.
      self.r_pos_j5 = 0.
      self.r_pos_j6 = 0.
      self.r_vel_j0 = 0.
      self.r_vel_j1 = 0.
      self.r_vel_j2 = 0.
      self.r_vel_j3 = 0.
      self.r_vel_j4 = 0.
      self.r_vel_j5 = 0.
      self.r_vel_j6 = 0.
      self.r_acc_j0 = 0.
      self.r_acc_j1 = 0.
      self.r_acc_j2 = 0.
      self.r_acc_j3 = 0.
      self.r_acc_j4 = 0.
      self.r_acc_j5 = 0.
      self.r_acc_j6 = 0.
      self.r_eff_j0 = 0.
      self.r_eff_j1 = 0.
      self.r_eff_j2 = 0.
      self.r_eff_j3 = 0.
      self.r_eff_j4 = 0.
      self.r_eff_j5 = 0.
      self.r_eff_j6 = 0.
      self.l_limit_0 = 0.
      self.l_limit_1 = 0.
      self.l_limit_2 = 0.
      self.l_limit_3 = 0.
      self.l_limit_4 = 0.
      self.l_limit_5 = 0.
      self.l_limit_6 = 0.
      self.u_limit_0 = 0.
      self.u_limit_1 = 0.
      self.u_limit_2 = 0.
      self.u_limit_3 = 0.
      self.u_limit_4 = 0.
      self.u_limit_5 = 0.
      self.u_limit_6 = 0.
      self.kappa = 0.
      self.Kv = 0.
      self.lambda_ = 0.
      self.Kz = 0.
      self.Zb = 0.
      self.F = 0.
      self.G = 0.
      self.inParams = 0
      self.outParams = 0
      self.hiddenNodes = 0
      self.errorParams = 0
      self.feedForwardForce = 0.
      self.nn_ON = 0.
      self.cartPos_Kp_x = 0.
      self.cartPos_Kp_y = 0.
      self.cartPos_Kp_z = 0.
      self.cartPos_Kd_x = 0.
      self.cartPos_Kd_y = 0.
      self.cartPos_Kd_z = 0.
      self.cartRot_Kp_x = 0.
      self.cartRot_Kp_y = 0.
      self.cartRot_Kp_z = 0.
      self.cartRot_Kd_x = 0.
      self.cartRot_Kd_y = 0.
      self.cartRot_Kd_z = 0.
      self.useCurrentCartPose = False
      self.useNullspacePose = False
      self.cartIniX = 0.
      self.cartIniY = 0.
      self.cartIniZ = 0.
      self.cartIniRoll = 0.
      self.cartIniPitch = 0.
      self.cartIniYaw = 0.
      self.cartDesX = 0.
      self.cartDesY = 0.
      self.cartDesZ = 0.
      self.cartDesRoll = 0.
      self.cartDesPitch = 0.
      self.cartDesYaw = 0.
      self.m = 0.
      self.d = 0.
      self.k = 0.
      self.task_mA = 0.
      self.task_mB = 0.
      self.fixedFilterWeights = 0.
      self.w0 = 0.
      self.w1 = 0.
      self.w2 = 0.
      self.w3 = 0.
      self.w4 = 0.
      self.w5 = 0.
      self.w6 = 0.
      self.w7 = 0.

  def _get_types(self):
    """
    internal API method
    """
    return self._slot_types

  def serialize(self, buff):
    """
    serialize message into buffer
    :param buff: buffer, ``StringIO``
    """
    try:
      _x = self
      buff.write(_struct_142d4q14d2B26d.pack(_x.dt, _x.force_x, _x.force_y, _x.force_z, _x.torque_x, _x.torque_y, _x.torque_z, _x.acc_x, _x.acc_y, _x.acc_z, _x.r_eff_x, _x.r_eff_y, _x.r_eff_z, _x.r_trq_x, _x.r_trq_y, _x.r_trq_z, _x.reference_eff_j0, _x.reference_eff_j1, _x.reference_eff_j2, _x.reference_eff_j3, _x.reference_eff_j4, _x.reference_eff_j5, _x.reference_eff_j6, _x.taskRef_x, _x.taskRef_y, _x.taskRef_z, _x.taskRef_phi, _x.taskRef_theta, _x.taskRef_psi, _x.taskRefModel_x, _x.taskRefModel_y, _x.taskRefModel_z, _x.taskRefModel_phi, _x.taskRefModel_theta, _x.taskRefModel_psi, _x.m_cartPos_x, _x.m_cartPos_y, _x.m_cartPos_z, _x.m_cartPos_Qx, _x.m_cartPos_Qy, _x.m_cartPos_Qz, _x.m_cartPos_QW, _x.m_pos_x, _x.m_pos_y, _x.m_pos_z, _x.m_vel_x, _x.m_vel_y, _x.m_vel_z, _x.m_acc_x, _x.m_acc_y, _x.m_acc_z, _x.m_pos_j0, _x.m_pos_j1, _x.m_pos_j2, _x.m_pos_j3, _x.m_pos_j4, _x.m_pos_j5, _x.m_pos_j6, _x.m_vel_j0, _x.m_vel_j1, _x.m_vel_j2, _x.m_vel_j3, _x.m_vel_j4, _x.m_vel_j5, _x.m_vel_j6, _x.m_acc_j0, _x.m_acc_j1, _x.m_acc_j2, _x.m_acc_j3, _x.m_acc_j4, _x.m_acc_j5, _x.m_acc_j6, _x.m_eff_j0, _x.m_eff_j1, _x.m_eff_j2, _x.m_eff_j3, _x.m_eff_j4, _x.m_eff_j5, _x.m_eff_j6, _x.control_eff_j0, _x.control_eff_j1, _x.control_eff_j2, _x.control_eff_j3, _x.control_eff_j4, _x.control_eff_j5, _x.control_eff_j6, _x.r_cartPos_x, _x.r_cartPos_y, _x.r_cartPos_z, _x.r_cartPos_Qx, _x.r_cartPos_Qy, _x.r_cartPos_Qz, _x.r_cartPos_QW, _x.r_pos_j0, _x.r_pos_j1, _x.r_pos_j2, _x.r_pos_j3, _x.r_pos_j4, _x.r_pos_j5, _x.r_pos_j6, _x.r_vel_j0, _x.r_vel_j1, _x.r_vel_j2, _x.r_vel_j3, _x.r_vel_j4, _x.r_vel_j5, _x.r_vel_j6, _x.r_acc_j0, _x.r_acc_j1, _x.r_acc_j2, _x.r_acc_j3, _x.r_acc_j4, _x.r_acc_j5, _x.r_acc_j6, _x.r_eff_j0, _x.r_eff_j1, _x.r_eff_j2, _x.r_eff_j3, _x.r_eff_j4, _x.r_eff_j5, _x.r_eff_j6, _x.l_limit_0, _x.l_limit_1, _x.l_limit_2, _x.l_limit_3, _x.l_limit_4, _x.l_limit_5, _x.l_limit_6, _x.u_limit_0, _x.u_limit_1, _x.u_limit_2, _x.u_limit_3, _x.u_limit_4, _x.u_limit_5, _x.u_limit_6, _x.kappa, _x.Kv, _x.lambda_, _x.Kz, _x.Zb, _x.F, _x.G, _x.inParams, _x.outParams, _x.hiddenNodes, _x.errorParams, _x.feedForwardForce, _x.nn_ON, _x.cartPos_Kp_x, _x.cartPos_Kp_y, _x.cartPos_Kp_z, _x.cartPos_Kd_x, _x.cartPos_Kd_y, _x.cartPos_Kd_z, _x.cartRot_Kp_x, _x.cartRot_Kp_y, _x.cartRot_Kp_z, _x.cartRot_Kd_x, _x.cartRot_Kd_y, _x.cartRot_Kd_z, _x.useCurrentCartPose, _x.useNullspacePose, _x.cartIniX, _x.cartIniY, _x.cartIniZ, _x.cartIniRoll, _x.cartIniPitch, _x.cartIniYaw, _x.cartDesX, _x.cartDesY, _x.cartDesZ, _x.cartDesRoll, _x.cartDesPitch, _x.cartDesYaw, _x.m, _x.d, _x.k, _x.task_mA, _x.task_mB, _x.fixedFilterWeights, _x.w0, _x.w1, _x.w2, _x.w3, _x.w4, _x.w5, _x.w6, _x.w7))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize(self, str):
    """
    unpack serialized message in str into this message instance
    :param str: byte array of serialized message, ``str``
    """
    try:
      end = 0
      _x = self
      start = end
      end += 1490
      (_x.dt, _x.force_x, _x.force_y, _x.force_z, _x.torque_x, _x.torque_y, _x.torque_z, _x.acc_x, _x.acc_y, _x.acc_z, _x.r_eff_x, _x.r_eff_y, _x.r_eff_z, _x.r_trq_x, _x.r_trq_y, _x.r_trq_z, _x.reference_eff_j0, _x.reference_eff_j1, _x.reference_eff_j2, _x.reference_eff_j3, _x.reference_eff_j4, _x.reference_eff_j5, _x.reference_eff_j6, _x.taskRef_x, _x.taskRef_y, _x.taskRef_z, _x.taskRef_phi, _x.taskRef_theta, _x.taskRef_psi, _x.taskRefModel_x, _x.taskRefModel_y, _x.taskRefModel_z, _x.taskRefModel_phi, _x.taskRefModel_theta, _x.taskRefModel_psi, _x.m_cartPos_x, _x.m_cartPos_y, _x.m_cartPos_z, _x.m_cartPos_Qx, _x.m_cartPos_Qy, _x.m_cartPos_Qz, _x.m_cartPos_QW, _x.m_pos_x, _x.m_pos_y, _x.m_pos_z, _x.m_vel_x, _x.m_vel_y, _x.m_vel_z, _x.m_acc_x, _x.m_acc_y, _x.m_acc_z, _x.m_pos_j0, _x.m_pos_j1, _x.m_pos_j2, _x.m_pos_j3, _x.m_pos_j4, _x.m_pos_j5, _x.m_pos_j6, _x.m_vel_j0, _x.m_vel_j1, _x.m_vel_j2, _x.m_vel_j3, _x.m_vel_j4, _x.m_vel_j5, _x.m_vel_j6, _x.m_acc_j0, _x.m_acc_j1, _x.m_acc_j2, _x.m_acc_j3, _x.m_acc_j4, _x.m_acc_j5, _x.m_acc_j6, _x.m_eff_j0, _x.m_eff_j1, _x.m_eff_j2, _x.m_eff_j3, _x.m_eff_j4, _x.m_eff_j5, _x.m_eff_j6, _x.control_eff_j0, _x.control_eff_j1, _x.control_eff_j2, _x.control_eff_j3, _x.control_eff_j4, _x.control_eff_j5, _x.control_eff_j6, _x.r_cartPos_x, _x.r_cartPos_y, _x.r_cartPos_z, _x.r_cartPos_Qx, _x.r_cartPos_Qy, _x.r_cartPos_Qz, _x.r_cartPos_QW, _x.r_pos_j0, _x.r_pos_j1, _x.r_pos_j2, _x.r_pos_j3, _x.r_pos_j4, _x.r_pos_j5, _x.r_pos_j6, _x.r_vel_j0, _x.r_vel_j1, _x.r_vel_j2, _x.r_vel_j3, _x.r_vel_j4, _x.r_vel_j5, _x.r_vel_j6, _x.r_acc_j0, _x.r_acc_j1, _x.r_acc_j2, _x.r_acc_j3, _x.r_acc_j4, _x.r_acc_j5, _x.r_acc_j6, _x.r_eff_j0, _x.r_eff_j1, _x.r_eff_j2, _x.r_eff_j3, _x.r_eff_j4, _x.r_eff_j5, _x.r_eff_j6, _x.l_limit_0, _x.l_limit_1, _x.l_limit_2, _x.l_limit_3, _x.l_limit_4, _x.l_limit_5, _x.l_limit_6, _x.u_limit_0, _x.u_limit_1, _x.u_limit_2, _x.u_limit_3, _x.u_limit_4, _x.u_limit_5, _x.u_limit_6, _x.kappa, _x.Kv, _x.lambda_, _x.Kz, _x.Zb, _x.F, _x.G, _x.inParams, _x.outParams, _x.hiddenNodes, _x.errorParams, _x.feedForwardForce, _x.nn_ON, _x.cartPos_Kp_x, _x.cartPos_Kp_y, _x.cartPos_Kp_z, _x.cartPos_Kd_x, _x.cartPos_Kd_y, _x.cartPos_Kd_z, _x.cartRot_Kp_x, _x.cartRot_Kp_y, _x.cartRot_Kp_z, _x.cartRot_Kd_x, _x.cartRot_Kd_y, _x.cartRot_Kd_z, _x.useCurrentCartPose, _x.useNullspacePose, _x.cartIniX, _x.cartIniY, _x.cartIniZ, _x.cartIniRoll, _x.cartIniPitch, _x.cartIniYaw, _x.cartDesX, _x.cartDesY, _x.cartDesZ, _x.cartDesRoll, _x.cartDesPitch, _x.cartDesYaw, _x.m, _x.d, _x.k, _x.task_mA, _x.task_mB, _x.fixedFilterWeights, _x.w0, _x.w1, _x.w2, _x.w3, _x.w4, _x.w5, _x.w6, _x.w7,) = _struct_142d4q14d2B26d.unpack(str[start:end])
      self.useCurrentCartPose = bool(self.useCurrentCartPose)
      self.useNullspacePose = bool(self.useNullspacePose)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill


  def serialize_numpy(self, buff, numpy):
    """
    serialize message with numpy array types into buffer
    :param buff: buffer, ``StringIO``
    :param numpy: numpy python module
    """
    try:
      _x = self
      buff.write(_struct_142d4q14d2B26d.pack(_x.dt, _x.force_x, _x.force_y, _x.force_z, _x.torque_x, _x.torque_y, _x.torque_z, _x.acc_x, _x.acc_y, _x.acc_z, _x.r_eff_x, _x.r_eff_y, _x.r_eff_z, _x.r_trq_x, _x.r_trq_y, _x.r_trq_z, _x.reference_eff_j0, _x.reference_eff_j1, _x.reference_eff_j2, _x.reference_eff_j3, _x.reference_eff_j4, _x.reference_eff_j5, _x.reference_eff_j6, _x.taskRef_x, _x.taskRef_y, _x.taskRef_z, _x.taskRef_phi, _x.taskRef_theta, _x.taskRef_psi, _x.taskRefModel_x, _x.taskRefModel_y, _x.taskRefModel_z, _x.taskRefModel_phi, _x.taskRefModel_theta, _x.taskRefModel_psi, _x.m_cartPos_x, _x.m_cartPos_y, _x.m_cartPos_z, _x.m_cartPos_Qx, _x.m_cartPos_Qy, _x.m_cartPos_Qz, _x.m_cartPos_QW, _x.m_pos_x, _x.m_pos_y, _x.m_pos_z, _x.m_vel_x, _x.m_vel_y, _x.m_vel_z, _x.m_acc_x, _x.m_acc_y, _x.m_acc_z, _x.m_pos_j0, _x.m_pos_j1, _x.m_pos_j2, _x.m_pos_j3, _x.m_pos_j4, _x.m_pos_j5, _x.m_pos_j6, _x.m_vel_j0, _x.m_vel_j1, _x.m_vel_j2, _x.m_vel_j3, _x.m_vel_j4, _x.m_vel_j5, _x.m_vel_j6, _x.m_acc_j0, _x.m_acc_j1, _x.m_acc_j2, _x.m_acc_j3, _x.m_acc_j4, _x.m_acc_j5, _x.m_acc_j6, _x.m_eff_j0, _x.m_eff_j1, _x.m_eff_j2, _x.m_eff_j3, _x.m_eff_j4, _x.m_eff_j5, _x.m_eff_j6, _x.control_eff_j0, _x.control_eff_j1, _x.control_eff_j2, _x.control_eff_j3, _x.control_eff_j4, _x.control_eff_j5, _x.control_eff_j6, _x.r_cartPos_x, _x.r_cartPos_y, _x.r_cartPos_z, _x.r_cartPos_Qx, _x.r_cartPos_Qy, _x.r_cartPos_Qz, _x.r_cartPos_QW, _x.r_pos_j0, _x.r_pos_j1, _x.r_pos_j2, _x.r_pos_j3, _x.r_pos_j4, _x.r_pos_j5, _x.r_pos_j6, _x.r_vel_j0, _x.r_vel_j1, _x.r_vel_j2, _x.r_vel_j3, _x.r_vel_j4, _x.r_vel_j5, _x.r_vel_j6, _x.r_acc_j0, _x.r_acc_j1, _x.r_acc_j2, _x.r_acc_j3, _x.r_acc_j4, _x.r_acc_j5, _x.r_acc_j6, _x.r_eff_j0, _x.r_eff_j1, _x.r_eff_j2, _x.r_eff_j3, _x.r_eff_j4, _x.r_eff_j5, _x.r_eff_j6, _x.l_limit_0, _x.l_limit_1, _x.l_limit_2, _x.l_limit_3, _x.l_limit_4, _x.l_limit_5, _x.l_limit_6, _x.u_limit_0, _x.u_limit_1, _x.u_limit_2, _x.u_limit_3, _x.u_limit_4, _x.u_limit_5, _x.u_limit_6, _x.kappa, _x.Kv, _x.lambda_, _x.Kz, _x.Zb, _x.F, _x.G, _x.inParams, _x.outParams, _x.hiddenNodes, _x.errorParams, _x.feedForwardForce, _x.nn_ON, _x.cartPos_Kp_x, _x.cartPos_Kp_y, _x.cartPos_Kp_z, _x.cartPos_Kd_x, _x.cartPos_Kd_y, _x.cartPos_Kd_z, _x.cartRot_Kp_x, _x.cartRot_Kp_y, _x.cartRot_Kp_z, _x.cartRot_Kd_x, _x.cartRot_Kd_y, _x.cartRot_Kd_z, _x.useCurrentCartPose, _x.useNullspacePose, _x.cartIniX, _x.cartIniY, _x.cartIniZ, _x.cartIniRoll, _x.cartIniPitch, _x.cartIniYaw, _x.cartDesX, _x.cartDesY, _x.cartDesZ, _x.cartDesRoll, _x.cartDesPitch, _x.cartDesYaw, _x.m, _x.d, _x.k, _x.task_mA, _x.task_mB, _x.fixedFilterWeights, _x.w0, _x.w1, _x.w2, _x.w3, _x.w4, _x.w5, _x.w6, _x.w7))
    except struct.error as se: self._check_types(se)
    except TypeError as te: self._check_types(te)

  def deserialize_numpy(self, str, numpy):
    """
    unpack serialized message in str into this message instance using numpy for array types
    :param str: byte array of serialized message, ``str``
    :param numpy: numpy python module
    """
    try:
      end = 0
      _x = self
      start = end
      end += 1490
      (_x.dt, _x.force_x, _x.force_y, _x.force_z, _x.torque_x, _x.torque_y, _x.torque_z, _x.acc_x, _x.acc_y, _x.acc_z, _x.r_eff_x, _x.r_eff_y, _x.r_eff_z, _x.r_trq_x, _x.r_trq_y, _x.r_trq_z, _x.reference_eff_j0, _x.reference_eff_j1, _x.reference_eff_j2, _x.reference_eff_j3, _x.reference_eff_j4, _x.reference_eff_j5, _x.reference_eff_j6, _x.taskRef_x, _x.taskRef_y, _x.taskRef_z, _x.taskRef_phi, _x.taskRef_theta, _x.taskRef_psi, _x.taskRefModel_x, _x.taskRefModel_y, _x.taskRefModel_z, _x.taskRefModel_phi, _x.taskRefModel_theta, _x.taskRefModel_psi, _x.m_cartPos_x, _x.m_cartPos_y, _x.m_cartPos_z, _x.m_cartPos_Qx, _x.m_cartPos_Qy, _x.m_cartPos_Qz, _x.m_cartPos_QW, _x.m_pos_x, _x.m_pos_y, _x.m_pos_z, _x.m_vel_x, _x.m_vel_y, _x.m_vel_z, _x.m_acc_x, _x.m_acc_y, _x.m_acc_z, _x.m_pos_j0, _x.m_pos_j1, _x.m_pos_j2, _x.m_pos_j3, _x.m_pos_j4, _x.m_pos_j5, _x.m_pos_j6, _x.m_vel_j0, _x.m_vel_j1, _x.m_vel_j2, _x.m_vel_j3, _x.m_vel_j4, _x.m_vel_j5, _x.m_vel_j6, _x.m_acc_j0, _x.m_acc_j1, _x.m_acc_j2, _x.m_acc_j3, _x.m_acc_j4, _x.m_acc_j5, _x.m_acc_j6, _x.m_eff_j0, _x.m_eff_j1, _x.m_eff_j2, _x.m_eff_j3, _x.m_eff_j4, _x.m_eff_j5, _x.m_eff_j6, _x.control_eff_j0, _x.control_eff_j1, _x.control_eff_j2, _x.control_eff_j3, _x.control_eff_j4, _x.control_eff_j5, _x.control_eff_j6, _x.r_cartPos_x, _x.r_cartPos_y, _x.r_cartPos_z, _x.r_cartPos_Qx, _x.r_cartPos_Qy, _x.r_cartPos_Qz, _x.r_cartPos_QW, _x.r_pos_j0, _x.r_pos_j1, _x.r_pos_j2, _x.r_pos_j3, _x.r_pos_j4, _x.r_pos_j5, _x.r_pos_j6, _x.r_vel_j0, _x.r_vel_j1, _x.r_vel_j2, _x.r_vel_j3, _x.r_vel_j4, _x.r_vel_j5, _x.r_vel_j6, _x.r_acc_j0, _x.r_acc_j1, _x.r_acc_j2, _x.r_acc_j3, _x.r_acc_j4, _x.r_acc_j5, _x.r_acc_j6, _x.r_eff_j0, _x.r_eff_j1, _x.r_eff_j2, _x.r_eff_j3, _x.r_eff_j4, _x.r_eff_j5, _x.r_eff_j6, _x.l_limit_0, _x.l_limit_1, _x.l_limit_2, _x.l_limit_3, _x.l_limit_4, _x.l_limit_5, _x.l_limit_6, _x.u_limit_0, _x.u_limit_1, _x.u_limit_2, _x.u_limit_3, _x.u_limit_4, _x.u_limit_5, _x.u_limit_6, _x.kappa, _x.Kv, _x.lambda_, _x.Kz, _x.Zb, _x.F, _x.G, _x.inParams, _x.outParams, _x.hiddenNodes, _x.errorParams, _x.feedForwardForce, _x.nn_ON, _x.cartPos_Kp_x, _x.cartPos_Kp_y, _x.cartPos_Kp_z, _x.cartPos_Kd_x, _x.cartPos_Kd_y, _x.cartPos_Kd_z, _x.cartRot_Kp_x, _x.cartRot_Kp_y, _x.cartRot_Kp_z, _x.cartRot_Kd_x, _x.cartRot_Kd_y, _x.cartRot_Kd_z, _x.useCurrentCartPose, _x.useNullspacePose, _x.cartIniX, _x.cartIniY, _x.cartIniZ, _x.cartIniRoll, _x.cartIniPitch, _x.cartIniYaw, _x.cartDesX, _x.cartDesY, _x.cartDesZ, _x.cartDesRoll, _x.cartDesPitch, _x.cartDesYaw, _x.m, _x.d, _x.k, _x.task_mA, _x.task_mB, _x.fixedFilterWeights, _x.w0, _x.w1, _x.w2, _x.w3, _x.w4, _x.w5, _x.w6, _x.w7,) = _struct_142d4q14d2B26d.unpack(str[start:end])
      self.useCurrentCartPose = bool(self.useCurrentCartPose)
      self.useNullspacePose = bool(self.useNullspacePose)
      return self
    except struct.error as e:
      raise genpy.DeserializationError(e) #most likely buffer underfill

_struct_I = genpy.struct_I
_struct_142d4q14d2B26d = struct.Struct("<142d4q14d2B26d")
