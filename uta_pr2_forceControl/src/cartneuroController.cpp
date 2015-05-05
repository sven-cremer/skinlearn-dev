#include "uta_pr2_forceControl/cartneuroController.h"
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;

PR2CartneuroControllerClass::~PR2CartneuroControllerClass()
{
  sub_command_.shutdown();
}

/// Controller initialization in non-realtime
bool PR2CartneuroControllerClass::init(pr2_mechanism_model::RobotState *robot,
                                 ros::NodeHandle &n)
{
  // subscribe to wrench commands
  sub_command_ = n.subscribe<geometry_msgs::Wrench>
	("command", 1, &PR2CartneuroControllerClass::command, this);

//	// Verify that the version of the library that we linked against is
//	  // compatible with the version of the headers we compiled against.
//	  GOOGLE_PROTOBUF_VERIFY_VERSION;

  // Get the root and tip link names from parameter server.
  std::string root_name, tip_name;
  if (!n.getParam("root_name", root_name))
  {
    ROS_ERROR("No root name given in namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("tip_name", tip_name))
  {
    ROS_ERROR("No tip name given in namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  // Construct a chain from the root to the tip and prepare the kinematics.
  // Note the joints must be calibrated.
  if (!chain_.init(robot, root_name, tip_name))
  {
    ROS_ERROR("MyCartController could not use the chain from '%s' to '%s'",
              root_name.c_str(), tip_name.c_str());
    return false;
  }

  std::string gripper_acc_tip = "r_gripper_motor_accelerometer_link";

  if (!chain_acc_link.init(robot, root_name, gripper_acc_tip))
  {
    ROS_ERROR("MyCartController could not use the chain from '%s' to '%s'",
              root_name.c_str(), gripper_acc_tip.c_str());
    return false;
  }

  std::string urdf_param_ = "/robot_description";
  std::string urdf_string;

  if (!n.getParam(urdf_param_, urdf_string))
  {
    ROS_ERROR("URDF not loaded from parameter: %s)", urdf_param_.c_str());
    return false;
  }

  if (!urdf_model.initString(urdf_string))
  {
        ROS_ERROR("Failed to parse URDF file");
    return -1;
  }else {
        ROS_INFO("Successfully parsed URDF file");
  }

  std::string para_cartPos_Kp_x = "/cartPos_Kp_x";
  std::string para_cartPos_Kp_y = "/cartPos_Kp_y";
  std::string para_cartPos_Kp_z = "/cartPos_Kp_z";
  std::string para_cartPos_Kd_x = "/cartPos_Kd_x";
  std::string para_cartPos_Kd_y = "/cartPos_Kd_y";
  std::string para_cartPos_Kd_z = "/cartPos_Kd_z";

  std::string para_cartRot_Kp_x = "/cartRot_Kp_x";
  std::string para_cartRot_Kp_y = "/cartRot_Kp_y";
  std::string para_cartRot_Kp_z = "/cartRot_Kp_z";
  std::string para_cartRot_Kd_x = "/cartRot_Kd_x";
  std::string para_cartRot_Kd_y = "/cartRot_Kd_y";
  std::string para_cartRot_Kd_z = "/cartRot_Kd_z";

  cartPos_Kp_x = 0 ; cartRot_Kp_x = 0 ;
  cartPos_Kp_y = 0 ; cartRot_Kp_y = 0 ;
  cartPos_Kp_z = 0 ; cartRot_Kp_z = 0 ;
  cartPos_Kd_x = 0 ; cartRot_Kd_x = 0 ;
  cartPos_Kd_y = 0 ; cartRot_Kd_y = 0 ;
  cartPos_Kd_z = 0 ; cartRot_Kd_z = 0 ;

  if (!n.getParam( para_cartPos_Kp_x , cartPos_Kp_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kp_x.c_str()) ; return false; }
  if (!n.getParam( para_cartPos_Kp_y , cartPos_Kp_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kp_y.c_str()) ; return false; }
  if (!n.getParam( para_cartPos_Kp_z , cartPos_Kp_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kp_z.c_str()) ; return false; }
  if (!n.getParam( para_cartPos_Kd_x , cartPos_Kd_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kd_x.c_str()) ; return false; }
  if (!n.getParam( para_cartPos_Kd_y , cartPos_Kd_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kd_y.c_str()) ; return false; }
  if (!n.getParam( para_cartPos_Kd_z , cartPos_Kd_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kd_z.c_str()) ; return false; }

  if (!n.getParam( para_cartRot_Kp_x , cartRot_Kp_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kp_x.c_str()) ; return false; }
  if (!n.getParam( para_cartRot_Kp_y , cartRot_Kp_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kp_y.c_str()) ; return false; }
  if (!n.getParam( para_cartRot_Kp_z , cartRot_Kp_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kp_z.c_str()) ; return false; }
  if (!n.getParam( para_cartRot_Kd_x , cartRot_Kd_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kd_x.c_str()) ; return false; }
  if (!n.getParam( para_cartRot_Kd_y , cartRot_Kd_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kd_y.c_str()) ; return false; }
  if (!n.getParam( para_cartRot_Kd_z , cartRot_Kd_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kd_z.c_str()) ; return false; }


  // Store the robot handle for later use (to get time).
  robot_state_ = robot;

  // Construct the kdl solvers in non-realtime.
  chain_.toKDL(kdl_chain_);
  chain_acc_link.toKDL(kdl_chain_acc_link);

  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

  jnt_to_pose_solver_acc_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_acc_link));

  // Resize (pre-allocate) the variables in non-realtime.
  q_.resize(kdl_chain_.getNrOfJoints());
  q0_.resize(kdl_chain_.getNrOfJoints());
  qdot_.resize(kdl_chain_.getNrOfJoints());
  tau_c_.resize(kdl_chain_.getNrOfJoints());
  J_.resize(kdl_chain_.getNrOfJoints());

  qnom.resize(kdl_chain_.getNrOfJoints());
  q_lower.resize(kdl_chain_.getNrOfJoints());
  q_upper.resize(kdl_chain_.getNrOfJoints());
  qd_limit.resize(kdl_chain_.getNrOfJoints());

  q_lower(0) = urdf_model.getJoint("l_shoulder_pan_joint"  )->limits->lower;
  q_lower(1) = urdf_model.getJoint("l_shoulder_lift_joint" )->limits->lower;
  q_lower(2) = urdf_model.getJoint("l_upper_arm_roll_joint")->limits->lower;
  q_lower(3) = urdf_model.getJoint("l_elbow_flex_joint"    )->limits->lower;
  q_lower(4) = urdf_model.getJoint("l_forearm_roll_joint"  )->limits->lower;
  q_lower(5) = urdf_model.getJoint("l_wrist_flex_joint"    )->limits->lower;
  q_lower(6) = urdf_model.getJoint("l_wrist_roll_joint"    )->limits->lower;

  q_upper(0) = urdf_model.getJoint("l_shoulder_pan_joint"  )->limits->upper;
  q_upper(1) = urdf_model.getJoint("l_shoulder_lift_joint" )->limits->upper;
  q_upper(2) = urdf_model.getJoint("l_upper_arm_roll_joint")->limits->upper;
  q_upper(3) = urdf_model.getJoint("l_elbow_flex_joint"    )->limits->upper;
  q_upper(4) = urdf_model.getJoint("l_forearm_roll_joint"  )->limits->upper;
  q_upper(5) = urdf_model.getJoint("l_wrist_flex_joint"    )->limits->upper;
  q_upper(6) = urdf_model.getJoint("l_wrist_roll_joint"    )->limits->upper;

  // Since two joints are continuous
  q_upper(4) =   6.28 ;
  q_upper(6) =   6.28 ;

  q_lower(4) = - 6.28 ;
  q_lower(6) = - 6.28 ;

  qnom(0) = ( q_upper(0) - q_lower(0) ) / 2 ;
  qnom(1) = ( q_upper(1) - q_lower(1) ) / 2 ;
  qnom(2) = ( q_upper(2) - q_lower(2) ) / 2 ;
  qnom(3) = ( q_upper(3) - q_lower(3) ) / 2 ;
  qnom(4) = ( q_upper(4) - q_lower(4) ) / 2 ;
  qnom(5) = ( q_upper(5) - q_lower(5) ) / 2 ;
  qnom(6) = ( q_upper(6) - q_lower(6) ) / 2 ;

  Jacobian         = Eigen::MatrixXd::Zero( 6, kdl_chain_.getNrOfJoints() ) ;
  JacobianPinv     = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), 6 ) ;
  JacobianTrans    = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), 6 ) ;
  JacobianTransPinv= Eigen::MatrixXd::Zero( 6, kdl_chain_.getNrOfJoints() ) ;
  nullSpace        = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints() ) ;

  cartControlForce = Eigen::VectorXd::Zero( 6 ) ;
  nullspaceTorque  = Eigen::VectorXd::Zero( kdl_chain_.getNrOfJoints() ) ;
  controlTorque    = Eigen::VectorXd::Zero( kdl_chain_.getNrOfJoints() ) ;

//  ROS_ERROR("Joint no: %d", kdl_chain_.getNrOfJoints());

  std::string nn_kappa            = "/nn_kappa"            ;
  std::string nn_Kv               = "/nn_Kv"               ;
  std::string nn_lambda           = "/nn_lambda"           ;
  std::string nn_Kz               = "/nn_Kz"               ;
  std::string nn_Zb               = "/nn_Zb"               ;
  std::string nn_feedForwardForce = "/nn_feedForwardForce" ;
  std::string nn_nnF              = "/nn_nnF"              ;
  std::string nn_nnG              = "/nn_nnG"              ;
  std::string nn_ONparam          = "/nn_ON"               ;

  if (!n.getParam( nn_kappa            , kappa            ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_kappa.c_str())                        ; return false; }
  if (!n.getParam( nn_Kv               , Kv               ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_Kv.c_str())                           ; return false; }
  if (!n.getParam( nn_lambda           , lambda           ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_lambda.c_str())                       ; return false; }
  if (!n.getParam( nn_Kz               , Kz               ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_Kz.c_str())                           ; return false; }
  if (!n.getParam( nn_Zb               , Zb               ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_Zb.c_str())                           ; return false; }
  if (!n.getParam( nn_feedForwardForce , fFForce ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_feedForwardForce.c_str())             ; return false; }
  if (!n.getParam( nn_nnF              , nnF              ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_nnF.c_str())                          ; return false; }
  if (!n.getParam( nn_nnG              , nnG              ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_nnG.c_str())                          ; return false; }
  if (!n.getParam( nn_ONparam          , nn_ON            ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_ONparam.c_str())                      ; return false; }

  std::string para_nnNum_Inputs  = "/nnNum_Inputs" ;
  std::string para_nnNum_Outputs = "/nnNum_Outputs" ;
  std::string para_nnNum_Hidden  = "/nnNum_Hidden" ;
  std::string para_nnNum_Error   = "/nnNum_Error" ;
  std::string para_nnNum_Joints  = "/nnNum_Joints" ;

  if (!n.getParam( para_nnNum_Inputs , num_Inputs  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Inputs .c_str()) ; return false; }
  if (!n.getParam( para_nnNum_Outputs, num_Outputs )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Outputs.c_str()) ; return false; }
  if (!n.getParam( para_nnNum_Hidden , num_Hidden  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Hidden .c_str()) ; return false; }
  if (!n.getParam( para_nnNum_Error  , num_Error   )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Error  .c_str()) ; return false; }
  if (!n.getParam( para_nnNum_Joints , num_Joints  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Joints .c_str()) ; return false; }

  // Circle rate 3 rad/sec
  circle_rate = 3  ;
  circleLlim  = 0  ;
  circleUlim  = 1.5;

  std::string para_circleRate  = "/circleRate" ;
  std::string para_circleLlim  = "/circleLlim" ;
  std::string para_circleUlim  = "/circleUlim" ;

  if (!n.getParam( para_circleRate , circle_rate )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_circleRate .c_str()) ; return false; }
  if (!n.getParam( para_circleLlim , circleLlim  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_circleLlim .c_str()) ; return false; }
  if (!n.getParam( para_circleUlim , circleUlim  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_circleUlim .c_str()) ; return false; }

  std::string para_m_M = "/m_M" ;
  std::string para_m_S = "/m_S" ;
  std::string para_m_D = "/m_D" ;

  if (!n.getParam( para_m_M , m_M )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_M .c_str()) ; return false; }
  if (!n.getParam( para_m_S , m_S )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_S .c_str()) ; return false; }
  if (!n.getParam( para_m_D , m_D )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_D .c_str()) ; return false; }

  std::string para_task_mA = "/task_mA" ;
  std::string para_task_mB = "/task_mB" ;

  if (!n.getParam( para_task_mA , task_mA )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_task_mA .c_str()) ; return false; }
  if (!n.getParam( para_task_mB , task_mB )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_task_mB .c_str()) ; return false; }

  // Desired cartesian pose
  cartDesX     = 0.0 ;
  cartDesY     = 0.0 ;
  cartDesZ     = 0.0 ;
  cartDesRoll  = 0.0 ;
  cartDesPitch = 0.0 ;
  cartDesYaw   = 0.0 ;

  std::string para_cartDesX     = "/cartDesX";
  std::string para_cartDesY     = "/cartDesY";
  std::string para_cartDesZ     = "/cartDesZ";
  std::string para_cartDesRoll  = "/cartDesRoll";
  std::string para_cartDesPitch = "/cartDesPitch";
  std::string para_cartDesYaw   = "/cartDesYaw";

  if (!n.getParam( para_cartDesX     , cartDesX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesX    .c_str()) ; return false; }
  if (!n.getParam( para_cartDesY     , cartDesY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesY    .c_str()) ; return false; }
  if (!n.getParam( para_cartDesZ     , cartDesZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesZ    .c_str()) ; return false; }
  if (!n.getParam( para_cartDesRoll  , cartDesRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesRoll .c_str()) ; return false; }
  if (!n.getParam( para_cartDesPitch , cartDesPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesPitch.c_str()) ; return false; }
  if (!n.getParam( para_cartDesYaw   , cartDesYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesYaw  .c_str()) ; return false; }

  // Initial cartesian pose
  cartIniX     = 0.7 ;
  cartIniY     = 0.3 ;
  cartIniZ     = 0.1 ;
  cartIniRoll  = 0.0 ;
  cartIniPitch = 0.0 ;
  cartIniYaw   = 0.0 ;

  std::string para_cartIniX     = "/cartIniX";
  std::string para_cartIniY     = "/cartIniY";
  std::string para_cartIniZ     = "/cartIniZ";
  std::string para_cartIniRoll  = "/cartIniRoll";
  std::string para_cartIniPitch = "/cartIniPitch";
  std::string para_cartIniYaw   = "/cartIniYaw";

  if (!n.getParam( para_cartIniX     , cartIniX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniX.c_str()) ; return false; }
  if (!n.getParam( para_cartIniY     , cartIniY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniY.c_str()) ; return false; }
  if (!n.getParam( para_cartIniZ     , cartIniZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniZ.c_str()) ; return false; }
  if (!n.getParam( para_cartIniRoll  , cartIniRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniRoll .c_str()) ; return false; }
  if (!n.getParam( para_cartIniPitch , cartIniPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniPitch.c_str()) ; return false; }
  if (!n.getParam( para_cartIniYaw   , cartIniYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniYaw  .c_str()) ; return false; }

  useCurrentCartPose = false ;
  std::string para_useCurrentCartPose     = "/useCurrentCartPose";
  if (!n.getParam( para_useCurrentCartPose, useCurrentCartPose )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useCurrentCartPose.c_str()) ; return false; }

  useNullspacePose = true ;
  std::string para_useNullspacePose     = "/useNullspacePose";
  if (!n.getParam( para_useNullspacePose, useNullspacePose )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useNullspacePose.c_str()) ; return false; }

  useFTinput = false ;
  std::string para_useFTinput   = "/useFTinput";
  if (!n.getParam( para_useFTinput, useFTinput )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useFTinput.c_str()) ; return false; }

  useARMAmodel = false ;
  std::string para_useARMAmodel = "/useARMAmodel";
  if (!n.getParam( para_useARMAmodel, useARMAmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useARMAmodel.c_str()) ; return false; }

  useCTARMAmodel = false ;
  std::string para_useCTARMAmodel = "/useCTARMAmodel";
  if (!n.getParam( para_useCTARMAmodel, useCTARMAmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useCTARMAmodel.c_str()) ; return false; }

  useFIRmodel = false ;
  std::string para_useFIRmodel = "/useFIRmodel";
  if (!n.getParam( para_useFIRmodel, useFIRmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useFIRmodel.c_str()) ; return false; }

  useMRACmodel = false ;
  std::string para_useMRACmodel = "/useMRACmodel";
  if (!n.getParam( para_useMRACmodel, useMRACmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useMRACmodel.c_str()) ; return false; }

  useMSDmodel = false ;
  std::string para_useMSDmodel = "/useMSDmodel";
  if (!n.getParam( para_useMSDmodel, useMSDmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useMSDmodel.c_str()) ; return false; }

  useIRLmodel = false ;
  std::string para_useIRLmodel = "/useIRLmodel";
  if (!n.getParam( para_useIRLmodel, useIRLmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useIRLmodel.c_str()) ; return false; }

  useDirectmodel = false ;
  std::string para_useDirectmodel = "/useDirectmodel";
  if (!n.getParam( para_useDirectmodel, useDirectmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useDirectmodel.c_str()) ; return false; }


  externalRefTraj = true ;
  std::string para_externalRefTraj = "/externalRefTraj";
  if (!n.getParam( para_externalRefTraj, externalRefTraj )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_externalRefTraj.c_str()) ; return false; }

  intentEst_delT = 0.1 ;
  std::string para_intentEst_delT = "/intentEst_delT";
  if (!n.getParam( para_intentEst_delT, intentEst_delT )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_intentEst_delT.c_str()) ; return false; }

  intentEst_M = 1.0 ;
  std::string para_intentEst_M = "/intentEst_M";
  if (!n.getParam( para_intentEst_M, intentEst_M )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_intentEst_M.c_str()) ; return false; }


  std::string para_forceCutOffX = "/forceCutOffX";
  std::string para_forceCutOffY = "/forceCutOffY";
  std::string para_forceCutOffZ = "/forceCutOffZ";

  if (!n.getParam( para_forceCutOffX , forceCutOffX )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffX.c_str()) ; return false; }
  if (!n.getParam( para_forceCutOffY , forceCutOffY )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffY.c_str()) ; return false; }
  if (!n.getParam( para_forceCutOffZ , forceCutOffZ )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffZ.c_str()) ; return false; }

  std::string para_forceTorqueOn = "/forceTorqueOn";
  if (!n.getParam( para_forceTorqueOn , forceTorqueOn )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceTorqueOn.c_str()) ; return false; }

  std::string para_useFlexiForce = "/useFlexiForce";
  if (!n.getParam( para_useFlexiForce , useFlexiForce )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useFlexiForce.c_str()) ; return false; }

  std::string para_filtW0        = "/filtW0"        ;
  std::string para_filtW1        = "/filtW1"        ;
  std::string para_filtW2        = "/filtW2"        ;
  std::string para_filtW3        = "/filtW3"        ;
  std::string para_filtW4        = "/filtW4"        ;
  std::string para_filtW5        = "/filtW5"        ;
  std::string para_filtW6        = "/filtW6"        ;
  std::string para_filtW7        = "/filtW7"        ;

  std::string para_flex_1_filtW0 = "/flex_1_filtW0" ;
  std::string para_flex_1_filtW1 = "/flex_1_filtW1" ;
  std::string para_flex_1_filtW2 = "/flex_1_filtW2" ;
  std::string para_flex_1_filtW3 = "/flex_1_filtW3" ;
  std::string para_flex_1_filtW4 = "/flex_1_filtW4" ;
  std::string para_flex_1_filtW5 = "/flex_1_filtW5" ;
  std::string para_flex_1_filtW6 = "/flex_1_filtW6" ;
  std::string para_flex_1_filtW7 = "/flex_1_filtW7" ;

  std::string para_flex_2_filtW0 = "/flex_2_filtW0" ;
  std::string para_flex_2_filtW1 = "/flex_2_filtW1" ;
  std::string para_flex_2_filtW2 = "/flex_2_filtW2" ;
  std::string para_flex_2_filtW3 = "/flex_2_filtW3" ;
  std::string para_flex_2_filtW4 = "/flex_2_filtW4" ;
  std::string para_flex_2_filtW5 = "/flex_2_filtW5" ;
  std::string para_flex_2_filtW6 = "/flex_2_filtW6" ;
  std::string para_flex_2_filtW7 = "/flex_2_filtW7" ;

  std::string para_flex_3_filtW0 = "/flex_3_filtW0" ;
  std::string para_flex_3_filtW1 = "/flex_3_filtW1" ;
  std::string para_flex_3_filtW2 = "/flex_3_filtW2" ;
  std::string para_flex_3_filtW3 = "/flex_3_filtW3" ;
  std::string para_flex_3_filtW4 = "/flex_3_filtW4" ;
  std::string para_flex_3_filtW5 = "/flex_3_filtW5" ;
  std::string para_flex_3_filtW6 = "/flex_3_filtW6" ;
  std::string para_flex_3_filtW7 = "/flex_3_filtW7" ;

  std::string para_flex_4_filtW0 = "/flex_4_filtW0" ;
  std::string para_flex_4_filtW1 = "/flex_4_filtW1" ;
  std::string para_flex_4_filtW2 = "/flex_4_filtW2" ;
  std::string para_flex_4_filtW3 = "/flex_4_filtW3" ;
  std::string para_flex_4_filtW4 = "/flex_4_filtW4" ;
  std::string para_flex_4_filtW5 = "/flex_4_filtW5" ;
  std::string para_flex_4_filtW6 = "/flex_4_filtW6" ;
  std::string para_flex_4_filtW7 = "/flex_4_filtW7" ;

  filtW0 = 0.0 ;  flex_1_filtW0 = 0.0 ;  flex_2_filtW0 = 0.0 ;  flex_3_filtW0 = 0.0 ;  flex_4_filtW0 = 0.0 ;
  filtW1 = 0.0 ;  flex_1_filtW1 = 0.0 ;  flex_2_filtW1 = 0.0 ;  flex_3_filtW1 = 0.0 ;  flex_4_filtW1 = 0.0 ;
  filtW2 = 0.0 ;  flex_1_filtW2 = 0.0 ;  flex_2_filtW2 = 0.0 ;  flex_3_filtW2 = 0.0 ;  flex_4_filtW2 = 0.0 ;
  filtW3 = 0.0 ;  flex_1_filtW3 = 0.0 ;  flex_2_filtW3 = 0.0 ;  flex_3_filtW3 = 0.0 ;  flex_4_filtW3 = 0.0 ;
  filtW4 = 0.0 ;  flex_1_filtW4 = 0.0 ;  flex_2_filtW4 = 0.0 ;  flex_3_filtW4 = 0.0 ;  flex_4_filtW4 = 0.0 ;
  filtW5 = 0.0 ;  flex_1_filtW5 = 0.0 ;  flex_2_filtW5 = 0.0 ;  flex_3_filtW5 = 0.0 ;  flex_4_filtW5 = 0.0 ;
  filtW6 = 0.0 ;  flex_1_filtW6 = 0.0 ;  flex_2_filtW6 = 0.0 ;  flex_3_filtW6 = 0.0 ;  flex_4_filtW6 = 0.0 ;
  filtW7 = 0.0 ;  flex_1_filtW7 = 0.0 ;  flex_2_filtW7 = 0.0 ;  flex_3_filtW7 = 0.0 ;  flex_4_filtW7 = 0.0 ;

  if (!n.getParam( para_filtW0        , filtW0        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW0       .c_str()) ; return false; }
  if (!n.getParam( para_filtW1        , filtW1        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW1       .c_str()) ; return false; }
  if (!n.getParam( para_filtW2        , filtW2        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW2       .c_str()) ; return false; }
  if (!n.getParam( para_filtW3        , filtW3        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW3       .c_str()) ; return false; }
  if (!n.getParam( para_filtW4        , filtW4        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW4       .c_str()) ; return false; }
  if (!n.getParam( para_filtW5        , filtW5        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW5       .c_str()) ; return false; }
  if (!n.getParam( para_filtW6        , filtW6        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW6       .c_str()) ; return false; }
  if (!n.getParam( para_filtW7        , filtW7        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW7       .c_str()) ; return false; }

  if (!n.getParam( para_flex_1_filtW0 , flex_1_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW0.c_str()) ; return false; }
  if (!n.getParam( para_flex_1_filtW1 , flex_1_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW1.c_str()) ; return false; }
  if (!n.getParam( para_flex_1_filtW2 , flex_1_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW2.c_str()) ; return false; }
  if (!n.getParam( para_flex_1_filtW3 , flex_1_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW3.c_str()) ; return false; }
  if (!n.getParam( para_flex_1_filtW4 , flex_1_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW4.c_str()) ; return false; }
  if (!n.getParam( para_flex_1_filtW5 , flex_1_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW5.c_str()) ; return false; }
  if (!n.getParam( para_flex_1_filtW6 , flex_1_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW6.c_str()) ; return false; }
  if (!n.getParam( para_flex_1_filtW7 , flex_1_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW7.c_str()) ; return false; }

  if (!n.getParam( para_flex_2_filtW0 , flex_2_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW0.c_str()) ; return false; }
  if (!n.getParam( para_flex_2_filtW1 , flex_2_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW1.c_str()) ; return false; }
  if (!n.getParam( para_flex_2_filtW2 , flex_2_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW2.c_str()) ; return false; }
  if (!n.getParam( para_flex_2_filtW3 , flex_2_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW3.c_str()) ; return false; }
  if (!n.getParam( para_flex_2_filtW4 , flex_2_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW4.c_str()) ; return false; }
  if (!n.getParam( para_flex_2_filtW5 , flex_2_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW5.c_str()) ; return false; }
  if (!n.getParam( para_flex_2_filtW6 , flex_2_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW6.c_str()) ; return false; }
  if (!n.getParam( para_flex_2_filtW7 , flex_2_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW7.c_str()) ; return false; }

  if (!n.getParam( para_flex_3_filtW0 , flex_3_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW0.c_str()) ; return false; }
  if (!n.getParam( para_flex_3_filtW1 , flex_3_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW1.c_str()) ; return false; }
  if (!n.getParam( para_flex_3_filtW2 , flex_3_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW2.c_str()) ; return false; }
  if (!n.getParam( para_flex_3_filtW3 , flex_3_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW3.c_str()) ; return false; }
  if (!n.getParam( para_flex_3_filtW4 , flex_3_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW4.c_str()) ; return false; }
  if (!n.getParam( para_flex_3_filtW5 , flex_3_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW5.c_str()) ; return false; }
  if (!n.getParam( para_flex_3_filtW6 , flex_3_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW6.c_str()) ; return false; }
  if (!n.getParam( para_flex_3_filtW7 , flex_3_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW7.c_str()) ; return false; }

  if (!n.getParam( para_flex_4_filtW0 , flex_4_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW0.c_str()) ; return false; }
  if (!n.getParam( para_flex_4_filtW1 , flex_4_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW1.c_str()) ; return false; }
  if (!n.getParam( para_flex_4_filtW2 , flex_4_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW2.c_str()) ; return false; }
  if (!n.getParam( para_flex_4_filtW3 , flex_4_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW3.c_str()) ; return false; }
  if (!n.getParam( para_flex_4_filtW4 , flex_4_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW4.c_str()) ; return false; }
  if (!n.getParam( para_flex_4_filtW5 , flex_4_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW5.c_str()) ; return false; }
  if (!n.getParam( para_flex_4_filtW6 , flex_4_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW6.c_str()) ; return false; }
  if (!n.getParam( para_flex_4_filtW7 , flex_4_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW7.c_str()) ; return false; }


  outerLoopWk.resize(8,1);
  outerLoopWk_flexi_1.resize(8,1);
  outerLoopWk_flexi_2.resize(8,1);
  outerLoopWk_flexi_3.resize(8,1);
  outerLoopWk_flexi_4.resize(8,1);


  if( useFIRmodel || useARMAmodel || useCTARMAmodel)
  {
	  outerLoopWk(0,0) = filtW0 ; outerLoopWk_flexi_1(0,0) = flex_1_filtW0 ; outerLoopWk_flexi_2(0,0) = flex_2_filtW0 ; outerLoopWk_flexi_3(0,0) = flex_3_filtW0 ; outerLoopWk_flexi_4(0,0) = flex_4_filtW0 ;
	  outerLoopWk(1,0) = filtW1 ; outerLoopWk_flexi_1(1,0) = flex_1_filtW1 ; outerLoopWk_flexi_2(1,0) = flex_2_filtW1 ; outerLoopWk_flexi_3(1,0) = flex_3_filtW1 ; outerLoopWk_flexi_4(1,0) = flex_4_filtW1 ;
	  outerLoopWk(2,0) = filtW2 ; outerLoopWk_flexi_1(2,0) = flex_1_filtW2 ; outerLoopWk_flexi_2(2,0) = flex_2_filtW2 ; outerLoopWk_flexi_3(2,0) = flex_3_filtW2 ; outerLoopWk_flexi_4(2,0) = flex_4_filtW2 ;
	  outerLoopWk(3,0) = filtW3 ; outerLoopWk_flexi_1(3,0) = flex_1_filtW3 ; outerLoopWk_flexi_2(3,0) = flex_2_filtW3 ; outerLoopWk_flexi_3(3,0) = flex_3_filtW3 ; outerLoopWk_flexi_4(3,0) = flex_4_filtW3 ;
	  outerLoopWk(4,0) = filtW4 ; outerLoopWk_flexi_1(4,0) = flex_1_filtW4 ; outerLoopWk_flexi_2(4,0) = flex_2_filtW4 ; outerLoopWk_flexi_3(4,0) = flex_3_filtW4 ; outerLoopWk_flexi_4(4,0) = flex_4_filtW4 ;
	  outerLoopWk(5,0) = filtW5 ; outerLoopWk_flexi_1(5,0) = flex_1_filtW5 ; outerLoopWk_flexi_2(5,0) = flex_2_filtW5 ; outerLoopWk_flexi_3(5,0) = flex_3_filtW5 ; outerLoopWk_flexi_4(5,0) = flex_4_filtW5 ;
	  outerLoopWk(6,0) = filtW6 ; outerLoopWk_flexi_1(6,0) = flex_1_filtW6 ; outerLoopWk_flexi_2(6,0) = flex_2_filtW6 ; outerLoopWk_flexi_3(6,0) = flex_3_filtW6 ; outerLoopWk_flexi_4(6,0) = flex_4_filtW6 ;
	  outerLoopWk(7,0) = filtW7 ; outerLoopWk_flexi_1(7,0) = flex_1_filtW7 ; outerLoopWk_flexi_2(7,0) = flex_2_filtW7 ; outerLoopWk_flexi_3(7,0) = flex_3_filtW7 ; outerLoopWk_flexi_4(7,0) = flex_4_filtW7 ;
  }else
  {
	  outerLoopWk(0,0) = 0.0 ; outerLoopWk_flexi_1(0,0) = 0.0 ; outerLoopWk_flexi_2(0,0) = 0.0 ; outerLoopWk_flexi_3(0,0) = 0.0 ; outerLoopWk_flexi_4(0,0) = 0.0 ;
	  outerLoopWk(1,0) = 0.0 ; outerLoopWk_flexi_1(1,0) = 0.0 ; outerLoopWk_flexi_2(1,0) = 0.0 ; outerLoopWk_flexi_3(1,0) = 0.0 ; outerLoopWk_flexi_4(1,0) = 0.0 ;
	  outerLoopWk(2,0) = 0.0 ; outerLoopWk_flexi_1(2,0) = 0.0 ; outerLoopWk_flexi_2(2,0) = 0.0 ; outerLoopWk_flexi_3(2,0) = 0.0 ; outerLoopWk_flexi_4(2,0) = 0.0 ;
	  outerLoopWk(3,0) = 0.0 ; outerLoopWk_flexi_1(3,0) = 0.0 ; outerLoopWk_flexi_2(3,0) = 0.0 ; outerLoopWk_flexi_3(3,0) = 0.0 ; outerLoopWk_flexi_4(3,0) = 0.0 ;
	  outerLoopWk(4,0) = 0.0 ; outerLoopWk_flexi_1(4,0) = 0.0 ; outerLoopWk_flexi_2(4,0) = 0.0 ; outerLoopWk_flexi_3(4,0) = 0.0 ; outerLoopWk_flexi_4(4,0) = 0.0 ;
	  outerLoopWk(5,0) = 0.0 ; outerLoopWk_flexi_1(5,0) = 0.0 ; outerLoopWk_flexi_2(5,0) = 0.0 ; outerLoopWk_flexi_3(5,0) = 0.0 ; outerLoopWk_flexi_4(5,0) = 0.0 ;
	  outerLoopWk(6,0) = 0.0 ; outerLoopWk_flexi_1(6,0) = 0.0 ; outerLoopWk_flexi_2(6,0) = 0.0 ; outerLoopWk_flexi_3(6,0) = 0.0 ; outerLoopWk_flexi_4(6,0) = 0.0 ;
	  outerLoopWk(7,0) = 0.0 ; outerLoopWk_flexi_1(7,0) = 0.0 ; outerLoopWk_flexi_2(7,0) = 0.0 ; outerLoopWk_flexi_3(7,0) = 0.0 ; outerLoopWk_flexi_4(7,0) = 0.0 ;
  }

  int numIrlSamples = 100;
  std::string para_numIrlSamples = "/numIrlSamples";
  if (!n.getParam( para_numIrlSamples , numIrlSamples )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_numIrlSamples.c_str()) ; return false; }

  int numIrlLsIter = 10;
  std::string para_numIrlLsIter = "/numIrlLsIter";
  if (!n.getParam( para_numIrlLsIter , numIrlLsIter )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_numIrlLsIter.c_str()) ; return false; }

  int numCartDof = 1;
  std::string para_numCartDof = "/numCartDof";
  if (!n.getParam( para_numCartDof , numCartDof )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_numCartDof.c_str()) ; return false; }

  bool irlOneshot = true;
  std::string para_irlOneshot = "/irlOneshot";
  if (!n.getParam( para_irlOneshot , irlOneshot )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_irlOneshot.c_str()) ; return false; }


  std::string para_fixedFilterWeights = "/fixedFilterWeights";
  if (!n.getParam( para_fixedFilterWeights , useFixedWeights )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_fixedFilterWeights.c_str()) ; return false; }

  double rls_lambda = 0.98 ;
  double rls_sigma  = 1000 ;

  std::string para_rls_lambda = "/rls_lambda";
  std::string para_rls_sigma  = "/rls_sigma";

  if (!n.getParam( para_rls_lambda , rls_lambda )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_rls_lambda.c_str()) ; return false; }
  if (!n.getParam( para_rls_sigma  , rls_sigma  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_rls_sigma .c_str()) ; return false; }

  double mrac_gamma_1 = 1 ;
  double mrac_gamma_2 = 1 ;
  double mrac_gamma_3 = 1 ;
  double mrac_gamma_4 = 1 ;
  double mrac_gamma_5 = 1 ;

  std::string para_mrac_gamma_1 = "/mrac_gamma_1";
  std::string para_mrac_gamma_2 = "/mrac_gamma_2";
  std::string para_mrac_gamma_3 = "/mrac_gamma_3";
  std::string para_mrac_gamma_4 = "/mrac_gamma_4";
  std::string para_mrac_gamma_5 = "/mrac_gamma_5";

  if (!n.getParam( para_mrac_gamma_1 , mrac_gamma_1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_1.c_str()) ; return false; }
  if (!n.getParam( para_mrac_gamma_2 , mrac_gamma_2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_2.c_str()) ; return false; }
  if (!n.getParam( para_mrac_gamma_3 , mrac_gamma_3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_3.c_str()) ; return false; }
  if (!n.getParam( para_mrac_gamma_4 , mrac_gamma_4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_4.c_str()) ; return false; }
  if (!n.getParam( para_mrac_gamma_5 , mrac_gamma_5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_5.c_str()) ; return false; }

  double mrac_P_m = 1 ;
  double mrac_P_h = 1 ;

  std::string para_mrac_P_m = "/mrac_P_m";
  std::string para_mrac_P_h = "/mrac_P_h";

  if (!n.getParam( para_mrac_P_m , mrac_P_m )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_P_m.c_str()) ; return false; }
  if (!n.getParam( para_mrac_P_h , mrac_P_h )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_P_h.c_str()) ; return false; }

  for (int i = 0; i < num_Joints; ++i)
      n.param("saturation/" + chain_.getJoint(i)->joint_->name, saturation_[i], 0.0);

  delT = 0.001;

  std::string para_outerLoopTime = "/outerLoop_time";
  if (!n.getParam( para_outerLoopTime , outerLoopTime )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_outerLoopTime.c_str()) ; return false; }

  intentLoopTime = outerLoopTime;
  std::string para_intentLoopTime = "/intentEst_time";
  if (!n.getParam( para_intentLoopTime , intentLoopTime )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_intentLoopTime.c_str()) ; return false; }

  useSimHuman = false ;
  std::string para_simHuman = "/useSimHuman";
  if (!n.getParam( para_simHuman , useSimHuman )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_simHuman.c_str()) ; return false; }

  std::string para_simHuman_a = "/simHuman_a" ;
  std::string para_simHuman_b = "/simHuman_b" ;

  if (!n.getParam( para_simHuman_a , simHuman_a )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_simHuman_a .c_str()) ; return false; }
  if (!n.getParam( para_simHuman_b , simHuman_b )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_simHuman_b .c_str()) ; return false; }

  // initial conditions
  ode_init_x[0 ] = 0.0;
  ode_init_x[1 ] = 0.0;
  ode_init_x[2 ] = 0.0;
  ode_init_x[3 ] = 0.0;

  /////////////////////////
  // System Model

  // FIXME remove below stuff
  num_Inputs  = 44 ;
  num_Outputs = 6  ; // 6 for 6 cart dof
//  num_Hidden  = 100;
  num_Error   = 6  ;
  num_Joints  = 7  ;

  kdl_temp_joint_.resize( num_Joints );
  eigen_temp_joint.resize( num_Joints,1 );

  q       .resize( num_Joints ) ;
  qd      .resize( num_Joints ) ;
  qdd     .resize( num_Joints ) ;

  q_m     .resize( num_Outputs ) ;
  qd_m    .resize( num_Outputs ) ;
  qdd_m   .resize( num_Outputs ) ;

  // desired Cartesian states
  X_m     .resize( num_Outputs ) ;
  Xd_m    .resize( num_Outputs ) ;
  Xdd_m   .resize( num_Outputs ) ;

  // Prev desired Cartesian states
  p_X_m     .resize( num_Outputs ) ;
  p_Xd_m    .resize( num_Outputs ) ;
  p_Xdd_m   .resize( num_Outputs ) ;

  // Cartesian states
  X       .resize( num_Outputs ) ;
  Xd      .resize( num_Outputs ) ;

  t_r     .resize( num_Outputs ) ;
  task_ref.resize( num_Outputs ) ;
  task_refModel_output.resize( num_Outputs ) ;
  tau     .resize( num_Outputs ) ;

  q        = Eigen::VectorXd::Zero( num_Joints ) ;
  qd       = Eigen::VectorXd::Zero( num_Joints ) ;
  qdd      = Eigen::VectorXd::Zero( num_Joints ) ;
  q_m      = Eigen::VectorXd::Zero( num_Joints ) ;
  qd_m     = Eigen::VectorXd::Zero( num_Joints ) ;
  qdd_m    = Eigen::VectorXd::Zero( num_Joints ) ;

  X_m      = Eigen::VectorXd::Zero( num_Outputs ) ;
  Xd_m     = Eigen::VectorXd::Zero( num_Outputs ) ;
  Xdd_m    = Eigen::VectorXd::Zero( num_Outputs ) ;
  X        = Eigen::VectorXd::Zero( num_Outputs ) ;
  Xd       = Eigen::VectorXd::Zero( num_Outputs ) ;

  p_X_m    = Eigen::VectorXd::Zero( num_Outputs ) ;
  p_Xd_m   = Eigen::VectorXd::Zero( num_Outputs ) ;
  p_Xdd_m  = Eigen::VectorXd::Zero( num_Outputs ) ;

  X_m(0)   = cartIniX     ;
  X_m(1)   = cartIniY     ;
  X_m(2)   = cartIniZ     ;
  X_m(3)   = cartIniRoll  ;
  X_m(4)   = cartIniPitch ;
  X_m(5)   = cartIniYaw   ;

  p_X_m    = X_m   ;
  p_Xd_m   = Xd_m  ;
  p_Xdd_m  = Xdd_m ;

  transformed_force = Eigen::Vector3d::Zero();
  r_acc_data          = Eigen::Vector3d::Zero();

  t_r                  = Eigen::VectorXd::Zero( num_Outputs ) ;
  task_ref             = Eigen::VectorXd::Zero( num_Outputs ) ;
  task_refModel_output = Eigen::VectorXd::Zero( num_Outputs ) ;
  tau                  = Eigen::VectorXd::Zero( num_Outputs ) ;
  force                = Eigen::VectorXd::Zero( num_Outputs ) ;
  // FIXME remove this hardcoded 4 value
  flexiForce           = Eigen::VectorXd::Zero( 4 ) ;

  // Initial Reference
  task_ref = X_m ;

  Jacobian         = Eigen::MatrixXd::Zero( num_Outputs, kdl_chain_.getNrOfJoints() ) ;
  JacobianPinv     = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), 6 ) ;
  JacobianTrans    = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), 6 ) ;
  JacobianTransPinv= Eigen::MatrixXd::Zero( num_Outputs, kdl_chain_.getNrOfJoints() ) ;
  nullSpace        = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints() ) ;

  cartControlForce = Eigen::VectorXd::Zero( num_Outputs ) ;
  nullspaceTorque  = Eigen::VectorXd::Zero( kdl_chain_.getNrOfJoints() ) ;
  controlTorque    = Eigen::VectorXd::Zero( kdl_chain_.getNrOfJoints() ) ;

  /////////////////////////
  // Outer Loop Init

  // MRAC
  outerLoopMRACmodelX.updateDelT( outerLoopTime );
  outerLoopMRACmodelX.updateAB( task_mA,
                                task_mB );
  outerLoopMRACmodelX.updateIni( cartIniX,
  		  	  	  	  	  	  	 cartIniX );
  outerLoopMRACmodelX.updateSimHuman( useSimHuman );
  outerLoopMRACmodelX.updateSimHuman( simHuman_a,
                                      simHuman_b );
  outerLoopMRACmodelX.updateGamma( mrac_gamma_1,
			                       mrac_gamma_2,
			                       mrac_gamma_3,
			                       mrac_gamma_4,
			                       mrac_gamma_5 ) ;
  outerLoopMRACmodelX.updateCov( mrac_P_m,
                                 mrac_P_h ) ;

  outerLoopMRACmodelY.updateDelT( outerLoopTime );
  outerLoopMRACmodelY.updateAB( task_mA,
                                task_mB );
  outerLoopMRACmodelY.updateIni( cartIniY,
		  	  	  	  	  	  	 cartIniY );
  outerLoopMRACmodelY.updateSimHuman( useSimHuman );
  outerLoopMRACmodelY.updateSimHuman( simHuman_a,
                                      simHuman_b );
  outerLoopMRACmodelY.updateGamma( mrac_gamma_1,
			                       mrac_gamma_2,
			                       mrac_gamma_3,
			                       mrac_gamma_4,
			                       mrac_gamma_5 ) ;
  outerLoopMRACmodelY.updateCov( mrac_P_m,
                                 mrac_P_h ) ;

  // RLS

  outerLoopRLSmodelX.updateDelT( outerLoopTime );
  outerLoopRLSmodelX.updateAB( task_mA,
                               task_mB );
  outerLoopRLSmodelX.initRls( rls_lambda, rls_sigma );
//  outerLoopRLSmodelX.initPos( cartIniX );


  outerLoopRLSmodelY.updateDelT( outerLoopTime );
  outerLoopRLSmodelY.updateAB( task_mA,
                               task_mB );
  outerLoopRLSmodelY.initRls( rls_lambda, rls_sigma );
//  outerLoopRLSmodelY.initPos( cartIniY );

  // CT RLS
  outerLoopCTRLSmodelX.updateDelT( outerLoopTime );
  outerLoopCTRLSmodelX.updateAB( task_mA,
                                 task_mB );

  outerLoopCTRLSmodelY.updateDelT( outerLoopTime );
  outerLoopCTRLSmodelY.updateAB( task_mA,
                                 task_mB );

  // MSD
  outerLoopMSDmodelX.updateDelT( outerLoopTime );
  outerLoopMSDmodelX.updateMsd( m_M,
                                m_S,
                                m_D );

  outerLoopMSDmodelY.updateDelT( outerLoopTime );
  outerLoopMSDmodelY.updateMsd( m_M,
                                m_S,
                                m_D );

  // IRL
  outerLoopIRLmodelX.init( numCartDof, numIrlSamples, numIrlLsIter, irlOneshot );
  outerLoopIRLmodelX.updateDelT( outerLoopTime );
  outerLoopIRLmodelX.updateMsd( m_M,
                                m_S,
                                m_D );
  outerLoopIRLmodelX.updateAB( task_mA,
                               task_mB );

  outerLoopIRLmodelY.init( numCartDof, numIrlSamples, numIrlLsIter, irlOneshot );
  outerLoopIRLmodelY.updateDelT( outerLoopTime );
  outerLoopIRLmodelY.updateMsd( m_M,
                                m_S,
                                m_D );
  outerLoopIRLmodelY.updateAB( task_mA,
                               task_mB );


  /////////////////////////

  // System Model END
  /////////////////////////


  /////////////////////////
  // NN

  nnController.changeNNstructure( num_Inputs  ,   // num_Inputs
                                  num_Outputs ,   // num_Outputs
                                  num_Hidden  ,   // num_Hidden
                                  num_Error   ,   // num_Error
                                  num_Outputs );  // num_Joints = num_Outputs for cart space

  Eigen::MatrixXd p_Kv     ;
  Eigen::MatrixXd p_lambda ;

  p_Kv     .resize( num_Outputs, 1 ) ;
  p_lambda .resize( num_Outputs, 1 ) ;

// Filtered error
// r = (qd_m - qd) + lambda*(q_m - q);
// Kv*r
// Kd*(qd_m - qd) + Kp*(q_m - q) = Kv*(qd_m - qd) + Kv*lambda*(q_m - q);
// Kv = Kd | Kv*lambda = Kp ... lambda = Kp/Kv = Kp/Kd

  p_Kv << cartPos_Kd_x ,
          cartPos_Kd_y ,
          cartPos_Kd_z ,
          cartRot_Kd_x ,
          cartRot_Kd_y ,
          cartRot_Kd_z ;

  p_lambda << cartPos_Kp_x / cartPos_Kd_x ,
              cartPos_Kp_y / cartPos_Kd_y ,
              cartPos_Kp_z / cartPos_Kd_z ,
              cartRot_Kp_x / cartRot_Kd_x ,
              cartRot_Kp_y / cartRot_Kd_y ,
              cartRot_Kp_z / cartRot_Kd_z ;

  nnController.init( kappa  ,
                     p_Kv     ,
                     p_lambda ,
                     Kz     ,
                     Zb     ,
                     fFForce,
                     nnF    ,
                     nnG    ,
                     nn_ON   );

  nnController.updateDelT( delT );

  // NN END
  /////////////////////////

  /* get a handle to the hardware interface */
   pr2_hardware_interface::HardwareInterface* hardwareInterface = robot->model_->hw_;
   if(!hardwareInterface)
       ROS_ERROR("Something wrong with the hardware interface pointer!");

   if( forceTorqueOn )
   {
     l_ft_handle_ = hardwareInterface->getForceTorque("l_gripper_motor");
     r_ft_handle_ = hardwareInterface->getForceTorque("r_gripper_motor");

     if( !l_ft_handle_ )
         ROS_ERROR("Something wrong with getting l_ft handle");
     if( !r_ft_handle_ )
         ROS_ERROR("Something wrong with getting r_ft handle");
   }

     /* get a handle to the left gripper accelerometer */
     l_accelerometer_handle_ = hardwareInterface->getAccelerometer("l_gripper_motor");
     if(!l_accelerometer_handle_)
         ROS_ERROR("Something wrong with getting accelerometer handle");

     // set to 1.5 kHz bandwidth (should be the default)
     l_accelerometer_handle_->command_.bandwidth_ = pr2_hardware_interface::AccelerometerCommand
     		                                                              ::BANDWIDTH_1500HZ;

     // set to +/- 8g range (0=2g,1=4g)
     l_accelerometer_handle_->command_.range_ = pr2_hardware_interface::AccelerometerCommand
                                                                      ::RANGE_8G;

     /* get a handle to the right gripper accelerometer */
     r_accelerometer_handle_ = hardwareInterface->getAccelerometer("r_gripper_motor");
     if(!r_accelerometer_handle_)
         ROS_ERROR("Something wrong with getting accelerometer handle");

     // set to 1.5 kHz bandwidth (should be the default)
     r_accelerometer_handle_->command_.bandwidth_ = pr2_hardware_interface::AccelerometerCommand
    		                                                              ::BANDWIDTH_1500HZ;

     // set to +/- 8g range (0=2g,1=4g)
     r_accelerometer_handle_->command_.range_ = pr2_hardware_interface::AccelerometerCommand
                                                                      ::RANGE_4G;

     l_accelerationObserver = new accelerationObserver(l_accelerometer_handle_);
     r_accelerationObserver = new accelerationObserver(r_accelerometer_handle_);



  /////////////////////////
  // DATA COLLECTION

  save_srv_                = n.advertiseService("save"               , &PR2CartneuroControllerClass::save                 , this);
  publish_srv_             = n.advertiseService("publish"            , &PR2CartneuroControllerClass::publish              , this);
  capture_srv_             = n.advertiseService("capture"            , &PR2CartneuroControllerClass::capture              , this);
  setRefTraj_srv_          = n.advertiseService("setRefTraj"         , &PR2CartneuroControllerClass::setRefTraj           , this);
  toggleFixedWeights_srv_  = n.advertiseService("toggleFixedWeights" , &PR2CartneuroControllerClass::toggleFixedWeights   , this);

  pubFTData_             = n.advertise< geometry_msgs::WrenchStamped             >( "FT_data"              , StoreLen);
  pubModelStates_        = n.advertise< sensor_msgs::JointState                  >( "model_joint_states"   , StoreLen);
  pubRobotStates_        = n.advertise< sensor_msgs::JointState                  >( "robot_joint_states"   , StoreLen);
  pubModelCartPos_       = n.advertise< geometry_msgs::PoseStamped               >( "model_cart_pos"       , StoreLen);
  pubRobotCartPos_       = n.advertise< geometry_msgs::PoseStamped               >( "robot_cart_pos"       , StoreLen);
  pubControllerParam_    = n.advertise< neuroadaptive_msgs::controllerParam    >( "controller_params"    , StoreLen);
  pubControllerFullData_ = n.advertise< neuroadaptive_msgs::controllerFullData >( "controllerFullData"   , StoreLen);

  storage_index_ = StoreLen;

  // DATA COLLECTION END
  /////////////////////////

  return true;
}

/// Controller startup in realtime
void PR2CartneuroControllerClass::starting()
{

  // Get the current joint values to compute the initial tip location.
  chain_.getPositions(q0_);
  jnt_to_pose_solver_->JntToCart(q0_, x0_);

  /////////////////////////
  // System Model

  // System Model END
  /////////////////////////

  // Initialize the phase of the circle as zero.
  circle_phase_ = 0.0;
  startCircleTraj = true;

  // Also reset the time-of-last-servo-cycle.
  last_time_     = robot_state_->getTime() ;
  start_time_    = robot_state_->getTime() ;
  outer_elapsed_ = robot_state_->getTime() ;

  if( forceTorqueOn )
  {
    // set FT sensor bias due to gravity
    std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
    l_ft_samples    = l_ftData_vector.size() - 1;
    l_ftBias.wrench = l_ftData_vector[l_ft_samples];

    std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
    r_ft_samples    = r_ftData_vector.size() - 1;
    r_ftBias.wrench = r_ftData_vector[r_ft_samples];
  }

}

/// Controller update loop in realtime
void PR2CartneuroControllerClass::update()
{
  // Read flexi force serial data
//  tacSerial->getDataArrayFromSerialPort( flexiForce );
  flexiForce(0) = flexiforce_wrench_desi_.force(0) ;
  flexiForce(1) = flexiforce_wrench_desi_.force(1) ;
  flexiForce(2) = flexiforce_wrench_desi_.force(2) ;
  flexiForce(3) = flexiforce_wrench_desi_.torque(0);

//             0
//
//     1   >   ^    >   3 y
//
//             2
//             x

  if( forceTorqueOn )
  {
    // retrieve right accelerometer data
    std::vector<geometry_msgs::Vector3> rightGripperAcc = r_accelerometer_handle_->state_.samples_;

    r_acc_data( 0 ) = r_accelerationObserver->aX_lp ; // threeAccs[threeAccs.size()-1].x ;
    r_acc_data( 1 ) = r_accelerationObserver->aY_lp ; // threeAccs[threeAccs.size()-1].y ;
    r_acc_data( 2 ) = r_accelerationObserver->aZ_lp ; // threeAccs[threeAccs.size()-1].z ;

//    rightGripperAcc.clear(); // Do we need this?

    std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
    l_ft_samples    = l_ftData_vector.size() - 1;
//      l_ftData.wrench = l_ftData_vector[l_ft_samples];
    l_ftData.wrench.force.x  = l_ftData_vector[l_ft_samples].force.x  - l_ftBias.wrench.force.x ;
    l_ftData.wrench.force.y  = l_ftData_vector[l_ft_samples].force.y  - l_ftBias.wrench.force.y ;
    l_ftData.wrench.force.z  = l_ftData_vector[l_ft_samples].force.z  - l_ftBias.wrench.force.z ;
    l_ftData.wrench.torque.x = l_ftData_vector[l_ft_samples].torque.x - l_ftBias.wrench.torque.x;
    l_ftData.wrench.torque.y = l_ftData_vector[l_ft_samples].torque.y - l_ftBias.wrench.torque.y;
    l_ftData.wrench.torque.z = l_ftData_vector[l_ft_samples].torque.z - l_ftBias.wrench.torque.z;

    std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
    r_ft_samples    = r_ftData_vector.size() - 1;
//      r_ftData.wrench = r_ftData_vector[r_ft_samples];
    r_ftData.wrench.force.x  =   ( r_ftData_vector[r_ft_samples].force.x  - r_ftBias.wrench.force.x  ) ;
    r_ftData.wrench.force.y  =   ( r_ftData_vector[r_ft_samples].force.y  - r_ftBias.wrench.force.y  ) ;
    r_ftData.wrench.force.z  =   ( r_ftData_vector[r_ft_samples].force.z  - r_ftBias.wrench.force.z  ) ;
    r_ftData.wrench.torque.x =   ( r_ftData_vector[r_ft_samples].torque.x - r_ftBias.wrench.torque.x ) ;
    r_ftData.wrench.torque.y =   ( r_ftData_vector[r_ft_samples].torque.y - r_ftBias.wrench.torque.y ) ;
    r_ftData.wrench.torque.z =   ( r_ftData_vector[r_ft_samples].torque.z - r_ftBias.wrench.torque.z ) ;
  }

  double dt;                    // Servo loop time step

  // Calculate the dt between servo cycles.
  dt = (robot_state_->getTime() - last_time_).toSec();
  last_time_ = robot_state_->getTime();

  // Get the current joint positions and velocities.
  chain_.getPositions(q_);
  chain_.getVelocities(qdot_);

  // Compute the forward kinematics and Jacobian (at this location).
  jnt_to_pose_solver_->JntToCart(q_, x_);
  jnt_to_jac_solver_->JntToJac(q_, J_);

  jnt_to_pose_solver_acc_->JntToCart(q_, x_gripper_acc_);

  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    xdot_(i) = 0;
    for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++)
    {
      xdot_(i) += J_(i,j) * qdot_.qdot(j);
      // Eigen Jacobian
      Jacobian(i,j) = J_(i,j);
    }
  }

  ///////////////////////////////
  // Human force input
  // Force error

  /////////////////////////
  // Flexiforce sensor ////

  // X axis
  if( flexiForce(0) > flexiForce(2) ){
	  FLEX_force(0) = -flexiForce(0);
  }else{
	  FLEX_force(0) =  flexiForce(2);
  }
  // Y axis
  if( flexiForce(1) > flexiForce(3) ){
	  FLEX_force(1) =  flexiForce(1);
  }else{
	  FLEX_force(1) = -flexiForce(3);
  }
  // Z axis
  FLEX_force(2) = 0 ;

  // Flexiforce sensor END ////

  /////////////////
  // FT sensor ////

  if( forceTorqueOn )
  {
	  //  Eigen::Vector3d forceFT( r_ftData.wrench.force.x, r_ftData.wrench.force.y, r_ftData.wrench.force.z );
	  Eigen::Vector3d forceFT( l_ftData.wrench.force.x, l_ftData.wrench.force.y, l_ftData.wrench.force.z );
	  //                               w       x       y      z
	  Eigen::Quaterniond ft_to_acc(0.579, -0.406, -0.579, 0.406);
	  FT_transformed_force = ft_to_acc._transformVector( forceFT );
	  FT_transformed_force(1) = - FT_transformed_force(1);
  }

  // FT sensor END ////

  if( useFlexiForce )
  {
	  transformed_force = FLEX_force;
  }else{
	  transformed_force = FT_transformed_force;
  }

  // Force threshold
  if( ( transformed_force(0) < forceCutOffX ) && ( transformed_force(0) > -forceCutOffX ) ){ transformed_force(0) = 0; }
  if( ( transformed_force(1) < forceCutOffY ) && ( transformed_force(1) > -forceCutOffY ) ){ transformed_force(1) = 0; }
  if( ( transformed_force(2) < forceCutOffZ ) && ( transformed_force(2) > -forceCutOffZ ) ){ transformed_force(2) = 0; }

  // Human force input END
  ///////////////////////////////

  // Follow a circle of 10cm at 3 rad/sec.
  circle_phase_ += 3.0 * dt;
  KDL::Vector  circle(cartDesX,cartDesY,cartDesZ);

  circle(1) = cartDesY * (cos(circle_phase_) - 1);
  circle(2) = cartDesZ * sin(circle_phase_)      ;

  if( useCurrentCartPose )
  {
    // Start from current
    xd_ = x0_;
  }else
  {
    // Start from specified
    xd_.p = KDL::Vector(cartIniX,cartIniY,cartIniZ);
  }

  xd_.p += circle;
  xd_.M =  KDL::Rotation::RPY( cartDesRoll, cartDesPitch, cartDesYaw );

  // Calculate a Cartesian restoring force.
  xerr_.vel = x_.p - xd_.p;
  xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
                     xd_.M.UnitY() * x_.M.UnitY() +
                     xd_.M.UnitZ() * x_.M.UnitZ());

  robotCartPos_.position.x    = x_.p(0);
  robotCartPos_.position.y    = x_.p(1);
  robotCartPos_.position.z    = x_.p(2);
  x_.M.GetQuaternion( robotCartPos_.orientation.x ,
                      robotCartPos_.orientation.y ,
                      robotCartPos_.orientation.z ,
                      robotCartPos_.orientation.w  );

  modelCartPos_.position.x    = xd_.p(0);
  modelCartPos_.position.y    = xd_.p(1);
  modelCartPos_.position.z    = xd_.p(2);
  xd_.M.GetQuaternion( modelCartPos_.orientation.x ,
                       modelCartPos_.orientation.y ,
                       modelCartPos_.orientation.z ,
                       modelCartPos_.orientation.w  );

  // Current joint positions and velocities
  q = JointKdl2Eigen( q_ );
  qd = JointVelKdl2Eigen( qdot_ );

  double circleAmpl = (circleUlim - circleLlim)/2 ;

  {
    double R, P, Y;
    xd_.M.GetRPY(R, P, Y);
//    X_m(0) = xd_.p(0);  Xd_m(0) = 0;  Xdd_m(0) = 0;
//    X_m(1) = xd_.p(1);  Xd_m(1) = 0;  Xdd_m(1) = 0;
//    X_m(2) = xd_.p(2);  Xd_m(2) = 0;  Xdd_m(2) = 0;

    X_m(3) = R       ;  Xd_m(3) = 0;  Xdd_m(3) = 0;
    X_m(4) = P       ;  Xd_m(4) = 0;  Xdd_m(4) = 0;
    X_m(5) = Y       ;  Xd_m(5) = 0;  Xdd_m(5) = 0;

    x_.M.GetRPY(R, P, Y);
    X(0)   = x_.p(0);   Xd(0)   = xdot_(0);
    X(1)   = x_.p(1);   Xd(1)   = xdot_(1);
    X(2)   = x_.p(2);   Xd(2)   = xdot_(2);
    X(3)   = R      ;   Xd(3)   = xdot_(3);
    X(4)   = P      ;   Xd(4)   = xdot_(4);
    X(5)   = Y      ;   Xd(5)   = xdot_(5);
  }


    /////////////////////////
    // System Model

    qnom(0) = -0.5   ;
    qnom(1) =  0 ;
    qnom(2) = -1.50   ;
    qnom(3) = -1.7   ;
    qnom(4) =  1.50   ;
    qnom(5) =  0 ;
    qnom(6) =  0 ;

  // USed to auto set cart pose
/*    if( (int) ceil( (robot_state_->getTime() - start_time_).toSec() ) % 3 == 0 )
    {
      task_ref(0) = cartDesX ;
      task_ref(1) = cartDesY ;
      task_ref(2) = cartDesZ ;
    }

    if( (int) ceil( (robot_state_->getTime() - start_time_).toSec() ) % 6 == 0 )
    {
      task_ref(0) = cartIniX ;
      task_ref(1) = cartIniY ;
      task_ref(2) = cartIniZ ;
    }*/

    if( !forceTorqueOn )
    {
      /////////////////////////
      // Simulated human model

/*
      // Open loop version
      ode_init_x[2] = task_ref(1); // q_r
      ode_init_x[3] = 0          ; // qd_r

      // Closed loop version
      // ode_init_x[2] = ( task_ref(1) - X_m(1) ); // q_r
      // ode_init_x[3] = 0                       ; // - Xd (1)                ; // qd_r

  //    boost::numeric::odeint::integrate( human_model , ode_init_x , 0.0 , delT , delT );
  //    transformed_force(1) = ode_init_x[0];

//      double T  = 0.18;
//      double Kp = 779 ;
//      double Kd = 288 ;
//       Bakur's values ?
      double T  = 1;
      double Kp = 1 ;
      double Kd = 0 ;

      double a_h = 10000;
      double b_h = a_h;

      // Reduced human model
      transformed_force (1) = transformed_force (1) + ode_init_x[1] * delT;
      
      //               ( Kp q_r           + Kd qd_r          - hf                    ) / T
      // ode_init_x[1] = ( Kp*ode_init_x[2] + Kd*ode_init_x[3] - transformed_force (1) ) / T ;
      ode_init_x[1] = b_h*task_ref(1) - a_h*transformed_force (1);


      ROS_ERROR_STREAM("USING Simulated human model");
*/

      // END Simulated human model
      /////////////////////////
    }


    if( !useFTinput )
    {
      transformed_force(0) = 0 ;
      transformed_force(1) = 0 ;
    }

    // OUTER Loop Update

    // Human Intent Estimation
    if( !externalRefTraj )
    {
    	if( ( robot_state_->getTime() - intent_elapsed_ ).toSec() >= intentLoopTime )
    	{
    		calcHumanIntentPos( transformed_force, task_ref, intentEst_delT, intentEst_M );

    		// Transform human intent to torso lift link
    		task_ref.x() = x_gripper_acc_.p.x() + task_ref.x() ;
    		task_ref.y() = x_gripper_acc_.p.y() + task_ref.y() ;
    		task_ref.z() = x_gripper_acc_.p.z() + task_ref.z() ;

    		intent_elapsed_ = robot_state_->getTime() ;
    	}
    }


    if( ( robot_state_->getTime() - outer_elapsed_ ).toSec() >= outerLoopTime )
    {
		// RLS ARMA
		if( useARMAmodel )
		{
			if( useFlexiForce )
			{
                // Set ARMA parameters
                // X axis
                if( flexiForce(0) > flexiForce(2) ){

                    outerLoopRLSmodelX.setWeights( outerLoopWk_flexi_1 ) ;
                    if( useFixedWeights )
                    	outerLoopRLSmodelX.setFixedWeights( outerLoopWk_flexi_1 );
			        else
			        	outerLoopRLSmodelX.setUpdatedWeights();

                }else{
                	outerLoopRLSmodelX.setWeights( outerLoopWk_flexi_3 ) ;
                	if( useFixedWeights )
                		outerLoopRLSmodelX.setFixedWeights( outerLoopWk_flexi_3 );
                    else
                    	outerLoopRLSmodelX.setUpdatedWeights();
                }

                // Y axis
                if( flexiForce(1) > flexiForce(3) ){
                	outerLoopRLSmodelY.setWeights( outerLoopWk_flexi_2 ) ;
                	if( useFixedWeights )
                		outerLoopRLSmodelY.setFixedWeights( outerLoopWk_flexi_2 );
                	else
                		outerLoopRLSmodelY.setUpdatedWeights();
                }else{
                	outerLoopRLSmodelY.setWeights( outerLoopWk_flexi_4 ) ;
                	if( useFixedWeights )
                		outerLoopRLSmodelY.setFixedWeights( outerLoopWk_flexi_4 );
                	else
                		outerLoopRLSmodelY.setUpdatedWeights();
                }
			}

            // X axis
	        outerLoopRLSmodelX.updateARMA( Xd_m                   (0) ,
	                                       Xd                     (0) ,
	                                       X_m                    (0) ,
	                                       X                      (0) ,
	                                       Xdd_m                  (0) ,
	                                       transformed_force      (0) ,
	                                       task_ref               (0) ,
	                                       task_refModel_output   (0)  );

		    // Y axis
		    outerLoopRLSmodelY.updateARMA( Xd_m                   (1) ,
                                           Xd                     (1) ,
                                           X_m                    (1) ,
                                           X                      (1) ,
                                           Xdd_m                  (1) ,
                                           transformed_force      (1) ,
                                           task_ref               (1) ,
                                           task_refModel_output   (1)  );

            // ROS_ERROR_STREAM("USING RLS ARMA");

            if( useFlexiForce )
            {
   	 	        // X axis
                if( flexiForce(0) > flexiForce(2) )
                {
                	outerLoopRLSmodelX.getWeights( outerLoopWk_flexi_1 ) ;
                }else{
                	outerLoopRLSmodelX.getWeights( outerLoopWk_flexi_3 ) ;
                }

                // Y axis
                if( flexiForce(1) > flexiForce(3) )
                {
                	outerLoopRLSmodelY.getWeights( outerLoopWk_flexi_2 ) ;
                }else{
                	outerLoopRLSmodelY.getWeights( outerLoopWk_flexi_4 ) ;
                }

            }else
            {
            	outerLoopRLSmodelX.getWeights( outerLoopWk ) ;
				outerLoopRLSmodelY.getWeights( outerLoopWk ) ;
            }
// Delete this
//		    if( outerLoopWk_flexi_1.norm() == 0 && outerLoopWk_flexi_3.norm() == 0 )
//		    {
//		    	X_m(0) =  cartIniX     ;
//		    }
//
//		    if(  outerLoopWk_flexi_2.norm() == 0 && outerLoopWk_flexi_4.norm() == 0  )
//		    {
//		    	X_m(1) =  cartIniY     ;
//		    }

		}

		// CT RLS ARMA
		if( useCTARMAmodel )
		{
	//      outerLoopCTRLSmodelX.updateARMA( Xd_m              (0) ,
	//                                       Xd                (0) ,
	//                                       X_m               (0) ,
	//                                       X                 (0) ,
	//                                       Xdd_m             (0) ,
	//                                       transformed_force (0) ,
	//                                       task_ref          (0) ,
	//                                       task_refModel     (0)  );

		  // Y axis
		  outerLoopCTRLSmodelY.updateARMA( Xd_m                   (1) ,
										   Xd                     (1) ,
										   X_m                    (1) ,
										   X                      (1) ,
										   Xdd_m                  (1) ,
										   transformed_force      (1) ,
										   task_ref               (1) ,
										   task_refModel_output   (1)  );

	//      ROS_ERROR_STREAM("USING CT RLS ARMA");

		    outerLoopCTRLSmodelY.getWeights( outerLoopWk ) ;
	//		outerLoopCTRLSmodelY.setWeights( outerLoopWk ) ;
		}

		// RLS FIR
		if( useFIRmodel )
		{
	//      outerLoopRLSmodelX.updateFIR( Xd_m              (0) ,
	//                                    Xd                (0) ,
	//                                    X_m               (0) ,
	//                                    X                 (0) ,
	//                                    Xdd_m             (0) ,
	//                                    transformed_force (0) ,
	//                                    task_ref          (0) ,
	//                                    task_refModel     (0)  );

		  // Y axis
		  outerLoopRLSmodelY.updateFIR( Xd_m                   (1) ,
										Xd                     (1) ,
										X_m                    (1) ,
										X                      (1) ,
										Xdd_m                  (1) ,
										transformed_force      (1) ,
										task_ref               (1) ,
										task_refModel_output   (1)  );
	//      ROS_ERROR_STREAM("USING RLS FIR");

			outerLoopRLSmodelY.getWeights( outerLoopWk ) ;
	//		outerLoopRLSmodelY.setWeights( outerLoopWk ) ;
		}
/*
		// MRAC
		if( useMRACmodel )
		{
	//      outerLoopMRACmodelX.update( Xd_m              (0) ,
	//                                  Xd                (0) ,
	//                                  X_m               (0) ,
	//                                  X                 (0) ,
	//                                  Xdd_m             (0) ,
	//                                  transformed_force (0) ,
	//                                  task_ref          (0) ,
	//                                  task_refModel     (0)  );

		  // Y axis
		  outerLoopMRACmodelY.update( Xd_m              (1) ,
									  Xd                (1) ,
									  X_m               (1) ,
									  X                 (1) ,
									  Xdd_m             (1) ,
									  transformed_force (1) ,
									  task_ref          (1) ,
									  task_refModel     (1)  );

	//      ROS_ERROR_STREAM("USING MRAC");
		}
*/
		// MSD
		if( useMSDmodel )
		{
	      // Cartesian space MSD model
	      outerLoopMSDmodelX.update( Xd_m             (0) ,
	                                 Xd               (0) ,
	                                 X_m              (0) ,
	                                 X                (0) ,
	                                 Xdd_m            (0) ,
	                                 transformed_force(0)  );

		  outerLoopMSDmodelY.update( Xd_m             (1) ,
									 Xd               (1) ,
									 X_m              (1) ,
									 X                (1) ,
									 Xdd_m            (1) ,
									 transformed_force(1)  );
	//      ROS_ERROR_STREAM("USING MSD");
		}

		// IRL
		if( useIRLmodel )
		{
	//    // Cartesian space IRL model
	//    outerLoopIRLmodelX.updateIRL( Xd_m              (0) ,
	//   	                            Xd                (0) ,
	//   	                            X_m               (0) ,
	//   	                            X                 (0) ,
	//   	                            Xdd_m             (0) ,
	//   	                            transformed_force (0) ,
	//   	                            task_ref          (0) ,
	//   	                            task_refModel     (0)  );

			outerLoopIRLmodelY.updateIRL( Xd_m                  (1) ,
					                      Xd                    (1) ,
					                      X_m                   (1) ,
					                      X                     (1) ,
					                      Xdd_m                 (1) ,
					                      transformed_force     (1) ,
					                      task_ref              (1) ,
					                      task_refModel_output  (1)  );


			// IRL
//			outerLoopIRLmodelX.getMsd( m_M,
//			                           m_S,
//			                           m_D );
			outerLoopIRLmodelY.getMsd( m_M,
			                           m_S,
									   m_D );

	//      ROS_ERROR_STREAM("USING MSD");
		}

		// Direct Model
		if( useDirectmodel )
		{
			// q_d
			X_m(1)= task_refModel_output(1)      ;
			Xd_m  = (X_m - p_X_m)/outerLoopTime  ;
			Xdd_m = (Xd_m - p_Xd_m)/outerLoopTime;
		}

        // MRAC
        if( useMRACmodel )
        {
//      outerLoopMRACmodelX.update( Xd_m              (0) ,
//                                  Xd                (0) ,
//                                  X_m               (0) ,
//                                  X                 (0) ,
//                                  Xdd_m             (0) ,
//                                  transformed_force (0) ,
//                                  task_ref          (0) ,
//                                  task_refModel     (0)  );

          // Y axis
          outerLoopMRACmodelY.update( Xd_m                 (1) ,
                                      Xd                   (1) ,
                                      X_m                  (1) ,
                                      X                    (1) ,
                                      Xdd_m                (1) ,
                                      transformed_force    (1) ,
                                      task_ref             (1) ,
                                      task_refModel_output (1)  );

//      ROS_ERROR_STREAM("USING MRAC");
        }

		p_X_m    = X_m   ;
		p_Xd_m   = Xd_m  ;
		p_Xdd_m  = Xdd_m ;

		outer_elapsed_ = robot_state_->getTime() ;

    }


    // System Model END
    /////////////////////////

//ROS_ERROR_STREAM("Joints: " << q.transpose());

    // FIXME only care about Y for now, need to check the orientation of other forces
    t_r(0) = transformed_force(0) ;
    t_r(1) = transformed_force(1) ;
//    t_r(2) = transformed_force(2) ;

  /////////////////////////
  // NN
    nnController.UpdateCart( X     ,
                             Xd    ,
                             X_m   ,
                             Xd_m  ,
                             Xdd_m ,
                             q     ,
                             qd    ,
                             t_r   ,
                             force  );
  // NN END
  /////////////////////////

//    for (unsigned int i = 0 ; i < 6 ; i++)
//      cartControlForce(i) = - Kp_(i) * xerr_(i) - Kd_(i) * xdot_(i);

    JacobianTrans = Jacobian.transpose();

    // ======== J psuedo-inverse and Nullspace computation

    double k_posture = 50.0;
    double jacobian_inverse_damping = 0.01;

    JointVec joint_dd_ff_;

    joint_dd_ff_(0) = 3.33   ;
    joint_dd_ff_(1) = 1.16   ;
    joint_dd_ff_(2) = 0.1    ;
    joint_dd_ff_(3) = 0.25   ;
    joint_dd_ff_(4) = 0.133  ;
    joint_dd_ff_(5) = 0.0727 ;
    joint_dd_ff_(6) = 0.0727 ;

    // Computes pseudo-inverse of J
    Eigen::Matrix<double,6,6> I6; I6.setIdentity();
    Eigen::Matrix<double,6,6> JJt_damped = Jacobian * JacobianTrans + jacobian_inverse_damping * I6;
    Eigen::Matrix<double,6,6> JJt_inv_damped = JJt_damped.inverse();
    Eigen::Matrix<double,7,6> J_pinv = JacobianTrans * JJt_inv_damped;

    // Computes the nullspace of J
    Eigen::Matrix<double,7,7> I; I.setIdentity();
    nullSpace = I - J_pinv * Jacobian;

    // ======== Posture control

    // Computes the desired joint torques for achieving the posture
    nullspaceTorque.setZero();

    {
      /*
      JointVec posture_err ;

      posture_err(0) = qnom(0) - q_(0) ;
      posture_err(1) = qnom(1) - q_(1) ;
      posture_err(2) = qnom(2) - q_(2) ;
      posture_err(3) = qnom(3) - q_(3) ;
      posture_err(4) = qnom(4) - q_(4) ;
      posture_err(5) = qnom(5) - q_(5) ;
      posture_err(6) = qnom(6) - q_(6) ;

      for (size_t j = 0; j < 7; ++j)
      {
        if (chain_.getJoint(j)->joint_->type == urdf::Joint::CONTINUOUS)
          posture_err[j] = angles::normalize_angle(posture_err[j]);
      }

      for (size_t j = 0; j < 7; ++j)
      {
        if (fabs(qnom(j) - 9999) < 1e-5)
          posture_err[j] = 0.0;
      }

      JointVec qdd_posture = k_posture * posture_err;

      // Add nullspace velocity
      qd_m = nullSpace*qdd_posture ;
      */

      JointVec q_jointLimit ;
      JointVec q_manipAbility ;

/*      double delQ;
      double rho = 0.1;
      double qTildeMax = 0;
      double qTildeMin = 0;*/

      for (size_t j = 0; j < 7; ++j)
      {
        ///////////////////
        // Liegeois
        // This is the Liegeois cost function from 1977
        q_jointLimit(j) = - (q(j) - qnom(j) )/( q.rows() * ( q_upper(j) - q_lower(j))) ;
        // END Liegeois
        ///////////////////
/*
        ///////////////////
        // Chaumette
        // This is the Chaumette cost function from 1996??
        // http://www.irisa.fr/lagadic/pdf/2001_itra_chaumette.pdf
        // Marchand, E.; Chaumette, F.; Rizzo, A.,
        // "Using the task function approach to avoid robot joint limits and kinematic singularities in visual servoing,"
        // Intelligent Robots and Systems '96, IROS 96, Proceedings of the 1996 IEEE/RSJ International Conference on ,
        // vol.3, no., pp.1083,1090 vol.3, 4-8 Nov 1996

        delQ = ( q_upper(j) - q_lower(j));
        qTildeMin = qTildeMin + rho*delQ;
        qTildeMax = qTildeMax - rho*delQ;

        if( q_(j) > qTildeMax )
        {
          q_jointLimit(j) = ( q_(j) - qTildeMax )/delQ ;
        }else
        if( q_(j) < qTildeMin )
        {
          q_jointLimit(j) = ( q_(j) - qTildeMin )/delQ ;
        }else
        {
          q_jointLimit(j) = 0 ;
        }
        // END Chaumette
        ///////////////////
*/

        ///////////////////
        // Manip modified Siciliano
        // Pg. 145
//        q_manipAbility = 0;
        // END Manip
        ///////////////////

      }

      nullspaceTorque = nullSpace*50*( q_jointLimit - 0.0*qd );

    }

    // dynamically consistent generalized inverse is defined to
    // J^T# = (J M^−1 J^T)^-1 JM^−1
    if ( useNullspacePose )
    {
      tau = JacobianTrans*force + nullspaceTorque;
    }else
    {
      tau = JacobianTrans*force;
    }

/*    // ======== Torque Saturation
      double sat_scaling = 1.0;
      for (int i = 0; i < num_Joints; ++i) {
        if (saturation_[i] > 0.0)
          sat_scaling = std::min(sat_scaling, fabs(saturation_[i] / tau[i]));
      }
      JointVec tau_sat = sat_scaling * tau;

  // Convert from Eigen to KDL
//      tau_c_ = JointEigen2Kdl( tau );

  tau_c_(0) = tau_sat(0);
  tau_c_(1) = tau_sat(1);
  tau_c_(2) = tau_sat(2);
  tau_c_(3) = tau_sat(3);
  tau_c_(4) = tau_sat(4);
  tau_c_(5) = tau_sat(5);
  tau_c_(6) = tau_sat(6);*/

  tau_c_ = JointEigen2Kdl( tau );

  // And finally send these torques out.
  chain_.setEfforts( tau_c_ );

  /////////////////////////
  // DATA COLLECTION

  bufferData( dt );

  // DATA COLLECTION END
  /////////////////////////


  // Acceleration estimator
  l_accelerationObserver->spin();
  r_accelerationObserver->spin();

}

void PR2CartneuroControllerClass::bufferData( double & dt )
{
        int index = storage_index_;
        if ((index >= 0) && (index < StoreLen))
        {
//                tf::PoseKDLToMsg(x_m_, modelCartPos_);
//                tf::PoseKDLToMsg(x_  , robotCartPos_);

          msgControllerFullData[index].dt                = dt                          ;

          // Force Data
          msgControllerFullData[index].force_x           = transformed_force(0)        ; // r_ftData.wrench.force.x     ;
          msgControllerFullData[index].force_y           = transformed_force(1)        ; // r_ftData.wrench.force.y     ;
          msgControllerFullData[index].force_z           = transformed_force(2)        ; // r_ftData.wrench.force.z     ;
          msgControllerFullData[index].torque_x          = 0                           ; // r_ftData.wrench.torque.x    ;
          msgControllerFullData[index].torque_y          = 0                           ; // r_ftData.wrench.torque.y    ;
          msgControllerFullData[index].torque_z          = 0                           ; // r_ftData.wrench.torque.z    ;

          msgControllerFullData[index].flexiforce_1      = flexiForce(0)               ;
          msgControllerFullData[index].flexiforce_2      = flexiForce(1)               ;
          msgControllerFullData[index].flexiforce_3      = flexiForce(2)               ;
          msgControllerFullData[index].flexiforce_4      = flexiForce(3)               ;

          msgControllerFullData[index].FT_force_x        = FT_transformed_force(0)     ;
          msgControllerFullData[index].FT_force_y        = FT_transformed_force(1)     ;
          msgControllerFullData[index].FT_force_z        = FT_transformed_force(2)     ;

          msgControllerFullData[index].acc_x             = r_acc_data(0)               ;
          msgControllerFullData[index].acc_y             = r_acc_data(1)               ;
          msgControllerFullData[index].acc_z             = r_acc_data(2)               ;

          // Input reference efforts(torques)
          msgControllerFullData[index].reference_eff_j0  = 0                           ; //t_r(0) ;
          msgControllerFullData[index].reference_eff_j1  = 0                           ; //t_r(1) ;
          msgControllerFullData[index].reference_eff_j2  = 0                           ; //t_r(2) ;
          msgControllerFullData[index].reference_eff_j3  = 0                           ; //t_r(3) ;
          msgControllerFullData[index].reference_eff_j4  = 0                           ; //t_r(4) ;
          msgControllerFullData[index].reference_eff_j5  = 0                           ; //t_r(5) ;
          msgControllerFullData[index].reference_eff_j6  = 0                           ; //t_r(6) ;

          // Cartesian task reference
          msgControllerFullData[index].taskRef_x         = task_ref(0)                 ;
          msgControllerFullData[index].taskRef_y         = task_ref(1)                 ;
          msgControllerFullData[index].taskRef_z         = task_ref(2)                 ;
          msgControllerFullData[index].taskRef_phi       = 0                           ;
          msgControllerFullData[index].taskRef_theta     = 0                           ;
          msgControllerFullData[index].taskRef_psi       = 0                           ;

          // Cartesian task reference
          msgControllerFullData[index].taskRefModel_x    = task_refModel_output(0)            ;
          msgControllerFullData[index].taskRefModel_y    = task_refModel_output(1)            ;
          msgControllerFullData[index].taskRefModel_z    = task_refModel_output(2)            ;
          msgControllerFullData[index].taskRefModel_phi  = 0                           ;
          msgControllerFullData[index].taskRefModel_theta= 0                           ;
          msgControllerFullData[index].taskRefModel_psi  = 0                           ;

          // Model States
          msgControllerFullData[index].m_cartPos_x       = modelCartPos_.position.x    ;
          msgControllerFullData[index].m_cartPos_y       = modelCartPos_.position.y    ;
          msgControllerFullData[index].m_cartPos_z       = modelCartPos_.position.z    ;
          msgControllerFullData[index].m_cartPos_Qx      = modelCartPos_.orientation.x ;
          msgControllerFullData[index].m_cartPos_Qy      = modelCartPos_.orientation.y ;
          msgControllerFullData[index].m_cartPos_Qz      = modelCartPos_.orientation.z ;
          msgControllerFullData[index].m_cartPos_QW      = modelCartPos_.orientation.w ;

          msgControllerFullData[index].m_pos_x           = X_m(0)                      ;
          msgControllerFullData[index].m_pos_y           = X_m(1)                      ;
          msgControllerFullData[index].m_pos_z           = X_m(2)                      ;

          msgControllerFullData[index].m_vel_x           = Xd_m(0)                     ;
          msgControllerFullData[index].m_vel_y           = Xd_m(1)                     ;
          msgControllerFullData[index].m_vel_z           = Xd_m(2)                     ;

          msgControllerFullData[index].m_acc_x           = Xdd_m(0)                    ;
          msgControllerFullData[index].m_acc_y           = Xdd_m(1)                    ;
          msgControllerFullData[index].m_acc_z           = Xdd_m(2)                    ;

          msgControllerFullData[index].m_pos_j0          = q_m(0)                      ;
          msgControllerFullData[index].m_pos_j1          = q_m(1)                      ;
          msgControllerFullData[index].m_pos_j2          = q_m(2)                      ;
          msgControllerFullData[index].m_pos_j3          = q_m(3)                      ;
          msgControllerFullData[index].m_pos_j4          = q_m(4)                      ;
          msgControllerFullData[index].m_pos_j5          = q_m(5)                      ;
          msgControllerFullData[index].m_pos_j6          = q_m(6)                      ;

          msgControllerFullData[index].m_vel_j0          = qd_m(0)                     ;
          msgControllerFullData[index].m_vel_j1          = qd_m(1)                     ;
          msgControllerFullData[index].m_vel_j2          = qd_m(2)                     ;
          msgControllerFullData[index].m_vel_j3          = qd_m(3)                     ;
          msgControllerFullData[index].m_vel_j4          = qd_m(4)                     ;
          msgControllerFullData[index].m_vel_j5          = qd_m(5)                     ;
          msgControllerFullData[index].m_vel_j6          = qd_m(6)                     ;

          msgControllerFullData[index].m_acc_j0          = qdd_m(0)                    ;
          msgControllerFullData[index].m_acc_j1          = qdd_m(1)                    ;
          msgControllerFullData[index].m_acc_j2          = qdd_m(2)                    ;
          msgControllerFullData[index].m_acc_j3          = qdd_m(3)                    ;
          msgControllerFullData[index].m_acc_j4          = qdd_m(4)                    ;
          msgControllerFullData[index].m_acc_j5          = qdd_m(5)                    ;
          msgControllerFullData[index].m_acc_j6          = qdd_m(6)                    ;

          msgControllerFullData[index].m_eff_j0          = 0                           ;
          msgControllerFullData[index].m_eff_j1          = 0                           ;
          msgControllerFullData[index].m_eff_j2          = 0                           ;
          msgControllerFullData[index].m_eff_j3          = 0                           ;
          msgControllerFullData[index].m_eff_j4          = 0                           ;
          msgControllerFullData[index].m_eff_j5          = 0                           ;
          msgControllerFullData[index].m_eff_j6          = 0                           ;

          // Control Output
          msgControllerFullData[index].control_eff_j0    = tau(0)                      ;
          msgControllerFullData[index].control_eff_j1    = tau(1)                      ;
          msgControllerFullData[index].control_eff_j2    = tau(2)                      ;
          msgControllerFullData[index].control_eff_j3    = tau(3)                      ;
          msgControllerFullData[index].control_eff_j4    = tau(4)                      ;
          msgControllerFullData[index].control_eff_j5    = tau(5)                      ;
          msgControllerFullData[index].control_eff_j6    = tau(6)                      ;

          // Robot States
          msgControllerFullData[index].r_cartPos_x       = robotCartPos_.position.x    ;
          msgControllerFullData[index].r_cartPos_y       = robotCartPos_.position.y    ;
          msgControllerFullData[index].r_cartPos_z       = robotCartPos_.position.z    ;
          msgControllerFullData[index].r_cartPos_Qx      = robotCartPos_.orientation.x ;
          msgControllerFullData[index].r_cartPos_Qy      = robotCartPos_.orientation.y ;
          msgControllerFullData[index].r_cartPos_Qz      = robotCartPos_.orientation.z ;
          msgControllerFullData[index].r_cartPos_QW      = robotCartPos_.orientation.w ;

          msgControllerFullData[index].r_pos_j0          = q(0)                        ;
          msgControllerFullData[index].r_pos_j1          = q(1)                        ;
          msgControllerFullData[index].r_pos_j2          = q(2)                        ;
          msgControllerFullData[index].r_pos_j3          = q(3)                        ;
          msgControllerFullData[index].r_pos_j4          = q(4)                        ;
          msgControllerFullData[index].r_pos_j5          = q(5)                        ;
          msgControllerFullData[index].r_pos_j6          = q(6)                        ;

          msgControllerFullData[index].r_vel_j0          = qd(0)                       ;
          msgControllerFullData[index].r_vel_j1          = qd(1)                       ;
          msgControllerFullData[index].r_vel_j2          = qd(2)                       ;
          msgControllerFullData[index].r_vel_j3          = qd(3)                       ;
          msgControllerFullData[index].r_vel_j4          = qd(4)                       ;
          msgControllerFullData[index].r_vel_j5          = qd(5)                       ;
          msgControllerFullData[index].r_vel_j6          = qd(6)                       ;

          msgControllerFullData[index].r_acc_j0          = 0                           ;
          msgControllerFullData[index].r_acc_j1          = 0                           ;
          msgControllerFullData[index].r_acc_j2          = 0                           ;
          msgControllerFullData[index].r_acc_j3          = 0                           ;
          msgControllerFullData[index].r_acc_j4          = 0                           ;
          msgControllerFullData[index].r_acc_j5          = 0                           ;
          msgControllerFullData[index].r_acc_j6          = 0                           ;
/*
          msgControllerFullData[index].r_eff_x           = ferr_(0)                    ;
          msgControllerFullData[index].r_eff_y           = ferr_(1)                    ;
          msgControllerFullData[index].r_eff_z           = ferr_(2)                    ;
          msgControllerFullData[index].r_trq_x           = ferr_(3)                    ;
          msgControllerFullData[index].r_trq_y           = ferr_(4)                    ;
          msgControllerFullData[index].r_trq_z           = ferr_(5)                    ;

          msgControllerFullData[index].r_eff_j0          = tau_f_(0)                   ;
          msgControllerFullData[index].r_eff_j1          = tau_f_(1)                   ;
          msgControllerFullData[index].r_eff_j2          = tau_f_(2)                   ;
          msgControllerFullData[index].r_eff_j3          = tau_f_(3)                   ;
          msgControllerFullData[index].r_eff_j4          = tau_f_(4)                   ;
          msgControllerFullData[index].r_eff_j5          = tau_f_(5)                   ;
          msgControllerFullData[index].r_eff_j6          = tau_f_(6)                   ;
*/
          msgControllerFullData[index].l_limit_0         = q_lower(0)                  ;
          msgControllerFullData[index].l_limit_1         = q_lower(1)                  ;
          msgControllerFullData[index].l_limit_2         = q_lower(2)                  ;
          msgControllerFullData[index].l_limit_3         = q_lower(3)                  ;
          msgControllerFullData[index].l_limit_4         = q_lower(4)                  ;
          msgControllerFullData[index].l_limit_5         = q_lower(5)                  ;
          msgControllerFullData[index].l_limit_6         = q_lower(6)                  ;

          msgControllerFullData[index].u_limit_0         = q_upper(0)                  ;
          msgControllerFullData[index].u_limit_1         = q_upper(1)                  ;
          msgControllerFullData[index].u_limit_2         = q_upper(2)                  ;
          msgControllerFullData[index].u_limit_3         = q_upper(3)                  ;
          msgControllerFullData[index].u_limit_4         = q_upper(4)                  ;
          msgControllerFullData[index].u_limit_5         = q_upper(5)                  ;
          msgControllerFullData[index].u_limit_6         = q_upper(6)                  ;

          // NN Params
          msgControllerFullData[index].kappa             = kappa                       ;
          msgControllerFullData[index].Kv                = Kv                          ;
          msgControllerFullData[index].lambda            = lambda                      ;
          msgControllerFullData[index].Kz                = Kz                          ;
          msgControllerFullData[index].Zb                = Zb                          ;
          msgControllerFullData[index].F                 = nnF                         ;
          msgControllerFullData[index].G                 = nnG                         ;
          msgControllerFullData[index].inParams          = num_Inputs                  ;
          msgControllerFullData[index].outParams         = num_Outputs                 ;
          msgControllerFullData[index].hiddenNodes       = num_Hidden                  ;
          msgControllerFullData[index].errorParams       = num_Error                   ;
          msgControllerFullData[index].feedForwardForce  = num_Joints                  ;
          msgControllerFullData[index].nn_ON             = nn_ON                       ;

          // Cart params
          msgControllerFullData[index].cartPos_Kp_x      = cartPos_Kp_x                ;
          msgControllerFullData[index].cartPos_Kp_y      = cartPos_Kp_y                ;
          msgControllerFullData[index].cartPos_Kp_z      = cartPos_Kp_z                ;
          msgControllerFullData[index].cartPos_Kd_x      = cartPos_Kd_x                ;
          msgControllerFullData[index].cartPos_Kd_y      = cartPos_Kd_y                ;
          msgControllerFullData[index].cartPos_Kd_z      = cartPos_Kd_z                ;

          msgControllerFullData[index].cartRot_Kp_x      = cartRot_Kp_x                ;
          msgControllerFullData[index].cartRot_Kp_y      = cartRot_Kp_y                ;
          msgControllerFullData[index].cartRot_Kp_z      = cartRot_Kp_z                ;
          msgControllerFullData[index].cartRot_Kd_x      = cartRot_Kd_x                ;
          msgControllerFullData[index].cartRot_Kd_y      = cartRot_Kd_y                ;
          msgControllerFullData[index].cartRot_Kd_z      = cartRot_Kd_z                ;

          msgControllerFullData[index].useCurrentCartPose= useCurrentCartPose          ;
          msgControllerFullData[index].useNullspacePose  = useNullspacePose            ;

          msgControllerFullData[index].cartIniX          = cartIniX                    ;
          msgControllerFullData[index].cartIniY          = cartIniY                    ;
          msgControllerFullData[index].cartIniZ          = cartIniZ                    ;
          msgControllerFullData[index].cartIniRoll       = cartIniRoll                 ;
          msgControllerFullData[index].cartIniPitch      = cartIniPitch                ;
          msgControllerFullData[index].cartIniYaw        = cartIniYaw                  ;

          msgControllerFullData[index].cartDesX          = cartDesX                    ;
          msgControllerFullData[index].cartDesY          = cartDesY                    ;
          msgControllerFullData[index].cartDesZ          = cartDesZ                    ;
          msgControllerFullData[index].cartDesRoll       = cartDesRoll                 ;
          msgControllerFullData[index].cartDesPitch      = cartDesPitch                ;
          msgControllerFullData[index].cartDesYaw        = cartDesYaw                  ;

          // TODO fix this
          // Model Params
          // 2nd degree ref model
          msgControllerFullData[index].m                 = m_M                         ; // outerLoopMSDmodel.getMass(  )(0,0) ;
          msgControllerFullData[index].d                 = m_D                         ; // outerLoopMSDmodel.getSpring()(0,0) ;
          msgControllerFullData[index].k                 = m_S                         ; // outerLoopMSDmodel.getDamper()(0,0) ;
          // 1st degree ref model
          msgControllerFullData[index].task_mA           = task_mA                     ;
          msgControllerFullData[index].task_mB           = task_mB                     ;

          msgControllerFullData[index].fixedFilterWeights= useFixedWeights             ;

          msgControllerFullData[index].w0                = outerLoopWk(0,0)            ;
          msgControllerFullData[index].w1                = outerLoopWk(1,0)            ;
          msgControllerFullData[index].w2                = outerLoopWk(2,0)            ;
          msgControllerFullData[index].w3                = outerLoopWk(3,0)            ;
          msgControllerFullData[index].w4                = outerLoopWk(4,0)            ;
          msgControllerFullData[index].w5                = outerLoopWk(5,0)            ;
          msgControllerFullData[index].w6                = outerLoopWk(6,0)            ;
          msgControllerFullData[index].w7                = outerLoopWk(7,0)            ;

          msgControllerFullData[index].f1_w0             = outerLoopWk_flexi_1(0,0)    ;
          msgControllerFullData[index].f1_w1             = outerLoopWk_flexi_1(1,0)    ;
          msgControllerFullData[index].f1_w2             = outerLoopWk_flexi_1(2,0)    ;
          msgControllerFullData[index].f1_w3             = outerLoopWk_flexi_1(3,0)    ;
          msgControllerFullData[index].f1_w4             = outerLoopWk_flexi_1(4,0)    ;
          msgControllerFullData[index].f1_w5             = outerLoopWk_flexi_1(5,0)    ;
          msgControllerFullData[index].f1_w6             = outerLoopWk_flexi_1(6,0)    ;
          msgControllerFullData[index].f1_w7             = outerLoopWk_flexi_1(7,0)    ;

          msgControllerFullData[index].f2_w0             = outerLoopWk_flexi_2(0,0)    ;
          msgControllerFullData[index].f2_w1             = outerLoopWk_flexi_2(1,0)    ;
          msgControllerFullData[index].f2_w2             = outerLoopWk_flexi_2(2,0)    ;
          msgControllerFullData[index].f2_w3             = outerLoopWk_flexi_2(3,0)    ;
          msgControllerFullData[index].f2_w4             = outerLoopWk_flexi_2(4,0)    ;
          msgControllerFullData[index].f2_w5             = outerLoopWk_flexi_2(5,0)    ;
          msgControllerFullData[index].f2_w6             = outerLoopWk_flexi_2(6,0)    ;
          msgControllerFullData[index].f2_w7             = outerLoopWk_flexi_2(7,0)    ;

          msgControllerFullData[index].f3_w0             = outerLoopWk_flexi_3(0,0)    ;
          msgControllerFullData[index].f3_w1             = outerLoopWk_flexi_3(1,0)    ;
          msgControllerFullData[index].f3_w2             = outerLoopWk_flexi_3(2,0)    ;
          msgControllerFullData[index].f3_w3             = outerLoopWk_flexi_3(3,0)    ;
          msgControllerFullData[index].f3_w4             = outerLoopWk_flexi_3(4,0)    ;
          msgControllerFullData[index].f3_w5             = outerLoopWk_flexi_3(5,0)    ;
          msgControllerFullData[index].f3_w6             = outerLoopWk_flexi_3(6,0)    ;
          msgControllerFullData[index].f3_w7             = outerLoopWk_flexi_3(7,0)    ;

          msgControllerFullData[index].f4_w0             = outerLoopWk_flexi_4(0,0)    ;
          msgControllerFullData[index].f4_w1             = outerLoopWk_flexi_4(1,0)    ;
          msgControllerFullData[index].f4_w2             = outerLoopWk_flexi_4(2,0)    ;
          msgControllerFullData[index].f4_w3             = outerLoopWk_flexi_4(3,0)    ;
          msgControllerFullData[index].f4_w4             = outerLoopWk_flexi_4(4,0)    ;
          msgControllerFullData[index].f4_w5             = outerLoopWk_flexi_4(5,0)    ;
          msgControllerFullData[index].f4_w6             = outerLoopWk_flexi_4(6,0)    ;
          msgControllerFullData[index].f4_w7             = outerLoopWk_flexi_4(7,0)    ;

          // MRAC Params
          outerLoopMRACmodelY.getGamma( msgControllerFullData[index].gamma_1,
                                        msgControllerFullData[index].gamma_2,
                                        msgControllerFullData[index].gamma_3,
                                        msgControllerFullData[index].gamma_4,
                                        msgControllerFullData[index].gamma_5 );

          outerLoopMRACmodelY.getEstimatedParams( msgControllerFullData[index].y_hat  ,
        	                                      msgControllerFullData[index].theta_1,
        	                                      msgControllerFullData[index].theta_2,
        	                                      msgControllerFullData[index].theta_3,
        	                                      msgControllerFullData[index].theta_4,
        	                                      msgControllerFullData[index].ahat   ,
        	                                      msgControllerFullData[index].bhat    );

          // Increment for the next cycle.
          storage_index_ = index+1;

/*          // Add a data point
          setDataPoint(controllerData.add_datum(), dt);*/
        }

/*        // Save to file
        {
              if (!controllerData.SerializeToOstream(&saveDataFile))
              {
                    ROS_ERROR_STREAM( "Failed to write data." );
              }
        }

            // Optional:  Delete all global objects allocated by libprotobuf.
            google::protobuf::ShutdownProtobufLibrary();*/
}

/*
void PR2CartneuroControllerClass::setDataPoint(dataPoint::Datum* datum, double & dt)
{
	datum->set_dt                 (  dt                          );
	datum->set_force_x            (  (double)transformed_force(0));
	datum->set_force_y            (  (double)transformed_force(1));
	datum->set_force_z            (  (double)transformed_force(2));
	datum->set_torque_x           (  0                           );
	datum->set_torque_y           (  0                           );
	datum->set_torque_z           (  0                           );
	datum->set_acc_x              (  (double)acc_data(0)         );
	datum->set_acc_y              (  (double)acc_data(1)         );
	datum->set_acc_z              (  (double)acc_data(2)         );
	datum->set_reference_eff_j0   (  0                           );
	datum->set_reference_eff_j1   (  0                           );
	datum->set_reference_eff_j2   (  0                           );
	datum->set_reference_eff_j3   (  0                           );
	datum->set_reference_eff_j4   (  0                           );
	datum->set_reference_eff_j5   (  0                           );
	datum->set_reference_eff_j6   (  0                           );
	datum->set_taskref_x          (  (double)task_ref(0)         );
	datum->set_taskref_y          (  (double)task_ref(1)         );
	datum->set_taskref_z          (  (double)task_ref(2)         );
	datum->set_taskref_phi        (  0                           );
	datum->set_taskref_theta      (  0                           );
	datum->set_taskref_psi        (  0                           );
	datum->set_taskrefmodel_x     (  (double)task_refModel(0)    );
	datum->set_taskrefmodel_y     (  (double)task_refModel(1)    );
	datum->set_taskrefmodel_z     (  (double)task_refModel(2)    );
	datum->set_taskrefmodel_phi   (  0                           );
	datum->set_taskrefmodel_theta (  0                           );
	datum->set_taskrefmodel_psi   (  0                           );
	datum->set_m_cartpos_x        (  modelCartPos_.position.x    );
	datum->set_m_cartpos_y        (  modelCartPos_.position.y    );
	datum->set_m_cartpos_z        (  modelCartPos_.position.z    );
	datum->set_m_cartpos_qx       (  modelCartPos_.orientation.x );
	datum->set_m_cartpos_qy       (  modelCartPos_.orientation.y );
	datum->set_m_cartpos_qz       (  modelCartPos_.orientation.z );
	datum->set_m_cartpos_qw       (  modelCartPos_.orientation.w );
	datum->set_m_pos_x            (  (double)X_m(0)              );
	datum->set_m_pos_y            (  (double)X_m(1)              );
	datum->set_m_pos_z            (  (double)X_m(2)              );
	datum->set_m_vel_x            (  (double)Xd_m(0)             );
	datum->set_m_vel_y            (  (double)Xd_m(1)             );
	datum->set_m_vel_z            (  (double)Xd_m(2)             );
	datum->set_m_acc_x            (  (double)Xdd_m(0)            );
	datum->set_m_acc_y            (  (double)Xdd_m(1)            );
	datum->set_m_acc_z            (  (double)Xdd_m(2)            );
	datum->set_m_pos_j0           (  (double)q_m(0)              );
	datum->set_m_pos_j1           (  (double)q_m(1)              );
	datum->set_m_pos_j2           (  (double)q_m(2)              );
	datum->set_m_pos_j3           (  (double)q_m(3)              );
	datum->set_m_pos_j4           (  (double)q_m(4)              );
	datum->set_m_pos_j5           (  (double)q_m(5)              );
	datum->set_m_pos_j6           (  (double)q_m(6)              );
	datum->set_m_vel_j0           (  (double)qd_m(0)             );
	datum->set_m_vel_j1           (  (double)qd_m(1)             );
	datum->set_m_vel_j2           (  (double)qd_m(2)             );
	datum->set_m_vel_j3           (  (double)qd_m(3)             );
	datum->set_m_vel_j4           (  (double)qd_m(4)             );
	datum->set_m_vel_j5           (  (double)qd_m(5)             );
	datum->set_m_vel_j6           (  (double)qd_m(6)             );
	datum->set_m_acc_j0           (  (double)qdd_m(0)            );
	datum->set_m_acc_j1           (  (double)qdd_m(1)            );
	datum->set_m_acc_j2           (  (double)qdd_m(2)            );
	datum->set_m_acc_j3           (  (double)qdd_m(3)            );
	datum->set_m_acc_j4           (  (double)qdd_m(4)            );
	datum->set_m_acc_j5           (  (double)qdd_m(5)            );
	datum->set_m_acc_j6           (  (double)qdd_m(6)            );
	datum->set_m_eff_j0           (  0                           );
	datum->set_m_eff_j1           (  0                           );
	datum->set_m_eff_j2           (  0                           );
	datum->set_m_eff_j3           (  0                           );
	datum->set_m_eff_j4           (  0                           );
	datum->set_m_eff_j5           (  0                           );
	datum->set_m_eff_j6           (  0                           );
	datum->set_control_eff_j0     (  (double)tau(0)              );
	datum->set_control_eff_j1     (  (double)tau(1)              );
	datum->set_control_eff_j2     (  (double)tau(2)              );
	datum->set_control_eff_j3     (  (double)tau(3)              );
	datum->set_control_eff_j4     (  (double)tau(4)              );
	datum->set_control_eff_j5     (  (double)tau(5)              );
	datum->set_control_eff_j6     (  (double)tau(6)              );
	datum->set_r_cartpos_x        (  robotCartPos_.position.x    );
	datum->set_r_cartpos_y        (  robotCartPos_.position.y    );
	datum->set_r_cartpos_z        (  robotCartPos_.position.z    );
	datum->set_r_cartpos_qx       (  robotCartPos_.orientation.x );
	datum->set_r_cartpos_qy       (  robotCartPos_.orientation.y );
	datum->set_r_cartpos_qz       (  robotCartPos_.orientation.z );
	datum->set_r_cartpos_qw       (  robotCartPos_.orientation.w );
	datum->set_r_pos_j0           (  (double)q(0)                );
	datum->set_r_pos_j1           (  (double)q(1)                );
	datum->set_r_pos_j2           (  (double)q(2)                );
	datum->set_r_pos_j3           (  (double)q(3)                );
	datum->set_r_pos_j4           (  (double)q(4)                );
	datum->set_r_pos_j5           (  (double)q(5)                );
	datum->set_r_pos_j6           (  (double)q(6)                );
	datum->set_r_vel_j0           (  (double)qd(0)               );
	datum->set_r_vel_j1           (  (double)qd(1)               );
	datum->set_r_vel_j2           (  (double)qd(2)               );
	datum->set_r_vel_j3           (  (double)qd(3)               );
	datum->set_r_vel_j4           (  (double)qd(4)               );
	datum->set_r_vel_j5           (  (double)qd(5)               );
	datum->set_r_vel_j6           (  (double)qd(6)               );
	datum->set_r_acc_j0           (  0                           );
	datum->set_r_acc_j1           (  0                           );
	datum->set_r_acc_j2           (  0                           );
	datum->set_r_acc_j3           (  0                           );
	datum->set_r_acc_j4           (  0                           );
	datum->set_r_acc_j5           (  0                           );
	datum->set_r_acc_j6           (  0                           );
//      datum->set_r_eff_x            (  ferr_(0)                    );
//	datum->set_r_eff_y            (  ferr_(1)                    );
//	datum->set_r_eff_z            (  ferr_(2)                    );
//	datum->set_r_trq_x            (  ferr_(3)                    );
//	datum->set_r_trq_y            (  ferr_(4)                    );
//	datum->set_r_trq_z            (  ferr_(5)                    );
//	datum->set_r_eff_j0           (  tau_f_(0)                   );
//	datum->set_r_eff_j1           (  tau_f_(1)                   );
//	datum->set_r_eff_j2           (  tau_f_(2)                   );
//	datum->set_r_eff_j3           (  tau_f_(3)                   );
//	datum->set_r_eff_j4           (  tau_f_(4)                   );
//	datum->set_r_eff_j5           (  tau_f_(5)                   );
//	datum->set_r_eff_j6           (  tau_f_(6)                   );
	datum->set_l_limit_0          (  q_lower(0)                  );
	datum->set_l_limit_1          (  q_lower(1)                  );
	datum->set_l_limit_2          (  q_lower(2)                  );
	datum->set_l_limit_3          (  q_lower(3)                  );
	datum->set_l_limit_4          (  q_lower(4)                  );
	datum->set_l_limit_5          (  q_lower(5)                  );
	datum->set_l_limit_6          (  q_lower(6)                  );
	datum->set_u_limit_0          (  q_upper(0)                  );
	datum->set_u_limit_1          (  q_upper(1)                  );
	datum->set_u_limit_2          (  q_upper(2)                  );
	datum->set_u_limit_3          (  q_upper(3)                  );
	datum->set_u_limit_4          (  q_upper(4)                  );
	datum->set_u_limit_5          (  q_upper(5)                  );
	datum->set_u_limit_6          (  q_upper(6)                  );
	datum->set_kappa              (  kappa                       );
	datum->set_kv                 (  Kv                          );
	datum->set_lambda             (  lambda                      );
	datum->set_kz                 (  Kz                          );
	datum->set_zb                 (  Zb                          );
	datum->set_f                  (  nnF                         );
	datum->set_g                  (  nnG                         );
	datum->set_inparams           (  num_Inputs                  );
	datum->set_outparams          (  num_Outputs                 );
	datum->set_hiddennodes        (  num_Hidden                  );
	datum->set_errorparams        (  num_Error                   );
	datum->set_feedforwardforce   (  num_Joints                  );
	datum->set_nn_on              (  nn_ON                       );
	datum->set_cartpos_kp_x       (  cartPos_Kp_x                );
	datum->set_cartpos_kp_y       (  cartPos_Kp_y                );
	datum->set_cartpos_kp_z       (  cartPos_Kp_z                );
	datum->set_cartpos_kd_x       (  cartPos_Kd_x                );
	datum->set_cartpos_kd_y       (  cartPos_Kd_y                );
	datum->set_cartpos_kd_z       (  cartPos_Kd_z                );
	datum->set_cartrot_kp_x       (  cartRot_Kp_x                );
	datum->set_cartrot_kp_y       (  cartRot_Kp_y                );
	datum->set_cartrot_kp_z       (  cartRot_Kp_z                );
	datum->set_cartrot_kd_x       (  cartRot_Kd_x                );
	datum->set_cartrot_kd_y       (  cartRot_Kd_y                );
	datum->set_cartrot_kd_z       (  cartRot_Kd_z                );
	datum->set_usecurrentcartpose (  useCurrentCartPose          );
	datum->set_usenullspacepose   (  useNullspacePose            );
	datum->set_cartinix           (  cartIniX                    );
	datum->set_cartiniy           (  cartIniY                    );
	datum->set_cartiniz           (  cartIniZ                    );
	datum->set_cartiniroll        (  cartIniRoll                 );
	datum->set_cartinipitch       (  cartIniPitch                );
	datum->set_cartiniyaw         (  cartIniYaw                  );
	datum->set_cartdesx           (  cartDesX                    );
	datum->set_cartdesy           (  cartDesY                    );
	datum->set_cartdesz           (  cartDesZ                    );
	datum->set_cartdesroll        (  cartDesRoll                 );
	datum->set_cartdespitch       (  cartDesPitch                );
	datum->set_cartdesyaw         (  cartDesYaw                  );
	datum->set_m                  (  m_M                         );
	datum->set_d                  (  m_D                         );
	datum->set_k                  (  m_S                         );
	datum->set_task_ma            (  task_mA                     );
	datum->set_task_mb            (  task_mB                     );
	datum->set_fixedfilterweights (  fixedFilterWeights          );
	datum->set_w0                 (  (double)outerLoopWk(0,0)    );
	datum->set_w1                 (  (double)outerLoopWk(1,0)    );
	datum->set_w2                 (  (double)outerLoopWk(2,0)    );
	datum->set_w3                 (  (double)outerLoopWk(3,0)    );
	datum->set_w4                 (  (double)outerLoopWk(4,0)    );
	datum->set_w5                 (  (double)outerLoopWk(5,0)    );
	datum->set_w6                 (  (double)outerLoopWk(6,0)    );
	datum->set_w7                 (  (double)outerLoopWk(7,0)    );

}
*/


/// Service call to set reference trajectory
bool PR2CartneuroControllerClass::setRefTraj( neuroadaptive_msgs::setCartPose::Request  & req ,
                                              neuroadaptive_msgs::setCartPose::Response & resp )
{
	if( externalRefTraj )
	{
        task_ref(0) = req.msg.position.x ;
        task_ref(1) = req.msg.position.y ;
        task_ref(2) = req.msg.position.z ;
	}

  resp.success = true;

  return true;
}

/// Service call to capture and extract the data
bool PR2CartneuroControllerClass::paramUpdate( neuroadaptive_msgs::controllerParamUpdate::Request  & req ,
                                               neuroadaptive_msgs::controllerParamUpdate::Response & resp )
{

  num_Inputs  = req.msg.inParams                 ;
  num_Outputs = req.msg.outParams                ;
  num_Hidden  = req.msg.hiddenNodes              ;
  num_Error   = req.msg.errorParams              ;

  kappa       = req.msg.kappa                    ;
  Kv          = req.msg.Kv                       ;
  lambda      = req.msg.lambda                   ;
  Kz          = req.msg.Kz                       ;
  Zb          = req.msg.Zb                       ;
  fFForce     = req.msg.feedForwardForce         ;
  nn_ON       = req.msg.nn_ON                    ;
  nnF         = req.msg.F                        ;
  nnG         = req.msg.G                        ;

  // FIXME actually update these in the
  // outer-loop controllers
  m_M         = req.msg.m                        ;
  m_D         = req.msg.d                        ;
  m_S         = req.msg.k                        ;

  task_mA     = req.msg.task_mA                  ;
  task_mB     = req.msg.task_mB                  ;

  // Cart params
  cartPos_Kp_x      = req.msg.cartPos_Kp_x       ;
  cartPos_Kp_y      = req.msg.cartPos_Kp_y       ;
  cartPos_Kp_z      = req.msg.cartPos_Kp_z       ;
  cartPos_Kd_x      = req.msg.cartPos_Kd_x       ;
  cartPos_Kd_y      = req.msg.cartPos_Kd_y       ;
  cartPos_Kd_z      = req.msg.cartPos_Kd_z       ;

  cartRot_Kp_x      = req.msg.cartRot_Kp_x       ;
  cartRot_Kp_y      = req.msg.cartRot_Kp_y       ;
  cartRot_Kp_z      = req.msg.cartRot_Kp_z       ;
  cartRot_Kd_x      = req.msg.cartRot_Kd_x       ;
  cartRot_Kd_y      = req.msg.cartRot_Kd_y       ;
  cartRot_Kd_z      = req.msg.cartRot_Kd_z       ;

  useCurrentCartPose= req.msg.useCurrentCartPose ;
  useNullspacePose  = req.msg.useNullspacePose   ;

  cartIniX          = req.msg.cartIniX           ;
  cartIniY          = req.msg.cartIniY           ;
  cartIniZ          = req.msg.cartIniZ           ;
  cartIniRoll       = req.msg.cartIniRoll        ;
  cartIniPitch      = req.msg.cartIniPitch       ;
  cartIniYaw        = req.msg.cartIniYaw         ;

  cartDesX          = req.msg.cartDesX           ;
  cartDesY          = req.msg.cartDesY           ;
  cartDesZ          = req.msg.cartDesZ           ;
  cartDesRoll       = req.msg.cartDesRoll        ;
  cartDesPitch      = req.msg.cartDesPitch       ;
  cartDesYaw        = req.msg.cartDesYaw         ;

  nnController.changeNNstructure( num_Inputs  ,   // num_Inputs
                                  num_Outputs ,   // num_Outputs
                                  num_Hidden  ,   // num_Hidden
                                  num_Error   ,   // num_Error
                                  num_Joints   ); // num_Joints

  Eigen::MatrixXd p_Kv     ;
  Eigen::MatrixXd p_lambda ;

  p_Kv     .resize( 6, 1 ) ;
  p_lambda .resize( 6, 1 ) ;

  p_Kv << cartPos_Kd_x ,
          cartPos_Kd_y ,
          cartPos_Kd_z ,
          cartRot_Kd_x ,
          cartRot_Kd_y ,
          cartRot_Kd_z ;

  p_lambda << cartPos_Kp_x / cartPos_Kd_x ,
              cartPos_Kp_y / cartPos_Kd_y ,
              cartPos_Kp_z / cartPos_Kd_z ,
              cartRot_Kp_x / cartRot_Kd_x ,
              cartRot_Kp_y / cartRot_Kd_y ,
              cartRot_Kp_z / cartRot_Kd_z ;

  nnController.init( kappa    ,
                     p_Kv     ,
                     p_lambda ,
                     Kz       ,
                     Zb       ,
                     fFForce  ,
                     nnF      ,
                     nnG      ,
                     nn_ON     );

  // MRAC
  outerLoopMRACmodelX.updateAB( task_mA,
                                task_mB );
  outerLoopMRACmodelY.updateAB( task_mA,
                                task_mB );

  // RLS
  outerLoopRLSmodelX.updateAB( task_mA,
                               task_mB );
  outerLoopRLSmodelY.updateAB( task_mA,
                               task_mB );

  // MSD
  outerLoopMSDmodelX.updateMsd( m_M,
                                m_S,
                                m_D );
  outerLoopMSDmodelY.updateMsd( m_M,
                                m_S,
                                m_D );

  // IRL
  outerLoopIRLmodelX.updateMsd( m_M,
                                m_S,
                                m_D );
  outerLoopIRLmodelY.updateMsd( m_M,
                                m_S,
                                m_D );

  resp.success = true;

  return true;
}

/// Service call to save the data
bool PR2CartneuroControllerClass::save( neuroadaptive_msgs::saveControllerData::Request & req,
		                                neuroadaptive_msgs::saveControllerData::Response& resp )
{
  /* Record the starting time. */
  ros::Time started = ros::Time::now();

  // Start circle traj
  startCircleTraj = true;

  /* Mark the buffer as clear (which will start storing). */
  storage_index_ = 0;

  std::string fileNameProto = req.fileName ;

  // Save data to file
  saveDataFile.open(fileNameProto.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);

  resp.success = true;

  return true;
}

/// Service call to publish the saved data
bool PR2CartneuroControllerClass::publish( std_srvs::Empty::Request & req,
                                           std_srvs::Empty::Response& resp )
{
  /* Then we can publish the buffer contents. */
  int  index;
  for (index = 0 ; index < StoreLen ; index++)
  {
//    pubFTData_         .publish(msgFTData         [index]);
//    pubModelStates_    .publish(msgModelStates    [index]);
//    pubRobotStates_    .publish(msgRobotStates    [index]);
//    pubModelCartPos_   .publish(msgModelCartPos   [index]);
//    pubRobotCartPos_   .publish(msgRobotCartPos   [index]);
//    pubControllerParam_.publish(msgControllerParam[index]);
      pubControllerFullData_.publish(msgControllerFullData[index]);
  }

  return true;
}


/// Service call to capture and extract the data
bool PR2CartneuroControllerClass::capture( std_srvs::Empty::Request & req,
                                           std_srvs::Empty::Response& resp )
{
  /* Record the starting time. */
  ros::Time started = ros::Time::now();

  // Start circle traj
  startCircleTraj = true;

  /* Mark the buffer as clear (which will start storing). */
  storage_index_ = 0;

  /* Now wait until the buffer is full. */
  while (storage_index_ < StoreLen)
        {
          /* Sleep for 1ms as not to hog the CPU. */
          ros::Duration(0.001).sleep();

          /* Make sure we don't hang here forever. */
          if (ros::Time::now() - started > ros::Duration(20))
                {
                  ROS_ERROR("Waiting for buffer to fill up took longer than 20 seconds!");
                  return false;
                }
        }

  // Start circle traj
  startCircleTraj = false;

  /* Then we can publish the buffer contents. */
  int  index;
  for (index = 0 ; index < StoreLen ; index++)
  {
//    pubFTData_         .publish(msgFTData         [index]);
//    pubModelStates_    .publish(msgModelStates    [index]);
//    pubRobotStates_    .publish(msgRobotStates    [index]);
//    pubModelCartPos_   .publish(msgModelCartPos   [index]);
//    pubRobotCartPos_   .publish(msgRobotCartPos   [index]);
//    pubControllerParam_.publish(msgControllerParam[index]);
          pubControllerFullData_.publish(msgControllerFullData[index]);
  }

  return true;
}

/// Service call to set reference trajectory
bool PR2CartneuroControllerClass::toggleFixedWeights( neuroadaptive_msgs::fixedWeightToggle::Request & req,
		                                              neuroadaptive_msgs::fixedWeightToggle::Response& resp )
{
  // XOR to toggle
  useFixedWeights =  ( useFixedWeights || true ) && !( useFixedWeights && true );
  resp.useFixedWeights = useFixedWeights ;
  // Filter Weights
  resp.w0 = filtW0 ; // outerLoopWk(0,0) ;
  resp.w1 = filtW1 ; // outerLoopWk(1,0) ;
  resp.w2 = filtW2 ; // outerLoopWk(2,0) ;
  resp.w3 = filtW3 ; // outerLoopWk(3,0) ;
  resp.w4 = filtW4 ; // outerLoopWk(4,0) ;
  resp.w5 = filtW5 ; // outerLoopWk(5,0) ;
  resp.w6 = filtW6 ; // outerLoopWk(6,0) ;
  resp.w7 = filtW7 ; // outerLoopWk(7,0) ;

  outerLoopWk(0,0) = filtW0 ;
  outerLoopWk(1,0) = filtW1 ;
  outerLoopWk(2,0) = filtW2 ;
  outerLoopWk(3,0) = filtW3 ;
  outerLoopWk(4,0) = filtW4 ;
  outerLoopWk(5,0) = filtW5 ;
  outerLoopWk(6,0) = filtW6 ;
  outerLoopWk(7,0) = filtW7 ;

  if( useFixedWeights )
  {
	  outerLoopRLSmodelY.setFixedWeights( outerLoopWk );
  }else
  {
	  outerLoopRLSmodelY.setUpdatedWeights();
  }

  std::vector<std::string> outerModel ;

  if( useARMAmodel   ){ outerModel.push_back("useARMAmodel"  ); }
  if( useCTARMAmodel ){ outerModel.push_back("useCTARMAmodel"); }
  if( useFIRmodel    ){ outerModel.push_back("useFIRmodel "  ); }
  if( useMRACmodel   ){ outerModel.push_back("useMRACmodel"  ); }
  if( useMSDmodel    ){ outerModel.push_back("useMSDmodel "  ); }

  if( useIRLmodel  )
  {
	  if( useFixedWeights )
	  {
		  outerModel.push_back("useIRLmodel : Update");
		  outerLoopIRLmodelY.setUpdateIrl();
	  }else
	  {
		  outerModel.push_back("useIRLmodel : Fixed");
		  outerLoopIRLmodelY.setFixedMsd();
	  }
  }

  resp.outerModel = outerModel;

  return true;
}

/// Controller stopping in realtime
void PR2CartneuroControllerClass::stopping()
{}

Eigen::MatrixXd
PR2CartneuroControllerClass::JointKdl2Eigen( KDL::JntArray & joint_ )
{
        eigen_temp_joint(0) = joint_(0);
        eigen_temp_joint(1) = joint_(1);
        eigen_temp_joint(2) = joint_(2);
        eigen_temp_joint(3) = joint_(3);
        eigen_temp_joint(4) = joint_(4);
        eigen_temp_joint(5) = joint_(5);
        eigen_temp_joint(6) = joint_(6);

        return eigen_temp_joint;
}

Eigen::MatrixXd
PR2CartneuroControllerClass::JointVelKdl2Eigen( KDL::JntArrayVel & joint_ )
{
        eigen_temp_joint(0) = joint_.qdot(0);
        eigen_temp_joint(1) = joint_.qdot(1);
        eigen_temp_joint(2) = joint_.qdot(2);
        eigen_temp_joint(3) = joint_.qdot(3);
        eigen_temp_joint(4) = joint_.qdot(4);
        eigen_temp_joint(5) = joint_.qdot(5);
        eigen_temp_joint(6) = joint_.qdot(6);

        return eigen_temp_joint;
}

KDL::JntArray
PR2CartneuroControllerClass::JointEigen2Kdl( Eigen::VectorXd & joint )
{
        kdl_temp_joint_(0) = joint(0);
        kdl_temp_joint_(1) = joint(1);
        kdl_temp_joint_(2) = joint(2);
        kdl_temp_joint_(3) = joint(3);
        kdl_temp_joint_(4) = joint(4);
        kdl_temp_joint_(5) = joint(5);
        kdl_temp_joint_(6) = joint(6);

        return kdl_temp_joint_;
}

void
PR2CartneuroControllerClass::calcHumanIntentPos( Eigen::Vector3d & force,
		                                         Eigen::VectorXd & pos,
		                                         double delT,
		                                         double m )
{
  Eigen::Vector3d intentPos = Eigen::Vector3d::Zero();
  Eigen::Vector3d intentVel = Eigen::Vector3d::Zero();
  Eigen::Vector3d intentAcc = Eigen::Vector3d::Zero();

  Eigen::Vector3d M(m, m, m);

  intentAcc = force.cwiseQuotient(M);

  intentVel = intentVel + intentAcc * delT ;
  intentPos = intentPos + intentVel * delT ;

  pos = intentPos;
}

void PR2CartneuroControllerClass::command(const geometry_msgs::WrenchConstPtr& wrench_msg)
{
  // convert to wrench command
  flexiforce_wrench_desi_.force(0) = wrench_msg->force.x;
  flexiforce_wrench_desi_.force(1) = wrench_msg->force.y;
  flexiforce_wrench_desi_.force(2) = wrench_msg->force.z;
  flexiforce_wrench_desi_.torque(0) = wrench_msg->torque.x;
  flexiforce_wrench_desi_.torque(1) = wrench_msg->torque.y;
  flexiforce_wrench_desi_.torque(2) = wrench_msg->torque.z;
}


// Register controller to pluginlib
//PLUGINLIB_REGISTER_CLASS( PR2CartneuroControllerClass,
//						  pr2_controller_ns::PR2CartneuroControllerClass,
//                          pr2_controller_interface::Controller )

PLUGINLIB_EXPORT_CLASS( pr2_controller_ns::PR2CartneuroControllerClass, pr2_controller_interface::Controller)
