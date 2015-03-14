#include "uta_pr2_forceControl/neuroadptController.h"
#include <pluginlib/class_list_macros.h>
#include <tf_conversions/tf_kdl.h>

using namespace pr2_controller_ns;

using namespace std;
using namespace boost::numeric::odeint;

/// Controller initialization in non-realtime
bool PR2NeuroadptControllerClass::init( pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n )
{
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
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_kappa.c_str())			 ; return false; }
  if (!n.getParam( nn_Kv               , Kv               ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_Kv.c_str())				 ; return false; }
  if (!n.getParam( nn_lambda           , lambda           ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_lambda.c_str())			 ; return false; }
  if (!n.getParam( nn_Kz               , Kz               ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_Kz.c_str())				 ; return false; }
  if (!n.getParam( nn_Zb               , Zb               ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_Zb.c_str())				 ; return false; }
  if (!n.getParam( nn_feedForwardForce , fFForce ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_feedForwardForce.c_str())             ; return false; }
  if (!n.getParam( nn_nnF              , nnF              ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_nnF.c_str())				 ; return false; }
  if (!n.getParam( nn_nnG              , nnG              ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_nnG.c_str())				 ; return false; }
  if (!n.getParam( nn_ONparam          , nn_ON            ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_ONparam.c_str())			 ; return false; }

  std::string para_m_M                 = "/m_M" ;
  std::string para_m_S                 = "/m_S" ;
  std::string para_m_D                 = "/m_D" ;

  if (!n.getParam( para_m_M           , m_M            ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_M.c_str())                          ; return false; }
  if (!n.getParam( para_m_S           , m_S            ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_S.c_str())                          ; return false; }
  if (!n.getParam( para_m_D           , m_D            ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_D.c_str())                      ; return false; }

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

  // Store the robot handle for later use (to get time).
  robot_state_ = robot;

  // Construct the kdl solvers in non-realtime.
  chain_.toKDL(kdl_chain_);
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));
//  pos_to_jnt_solver_.reset(new KDL::ChainIkSolverPos_LMA(kdl_chain_));
  vel_to_jnt_solver_.reset(new KDL::ChainIkSolverVel_wdls(kdl_chain_));
//  acc_to_jnt_solver_.reset(new KDL::ChainIkSolverAcc(kdl_chain_));

  // Resize (pre-allocate) the variables in non-realtime.
  q_.resize(kdl_chain_.getNrOfJoints());
  q0_.resize(kdl_chain_.getNrOfJoints());
  qdot_.resize(kdl_chain_.getNrOfJoints());

  kdl_qmdot_.resize(kdl_chain_.getNrOfJoints());

  tau_t_.resize(kdl_chain_.getNrOfJoints());
  tau_h_.resize(kdl_chain_.getNrOfJoints());
  tau_c_.resize(kdl_chain_.getNrOfJoints());
  tau_f_.resize(kdl_chain_.getNrOfJoints());

  q_m_.resize(kdl_chain_.getNrOfJoints());
  q0_m_.resize(kdl_chain_.getNrOfJoints());
  qd_m_.resize(kdl_chain_.getNrOfJoints());
  qdd_m_.resize(kdl_chain_.getNrOfJoints());

  q_lower.resize(kdl_chain_.getNrOfJoints());
  q_upper.resize(kdl_chain_.getNrOfJoints());
  qd_limit.resize(kdl_chain_.getNrOfJoints());
  q_nominal.resize(kdl_chain_.getNrOfJoints());

  joint_P   .resize(kdl_chain_.getNrOfJoints());
  joint_I   .resize(kdl_chain_.getNrOfJoints());
  joint_D   .resize(kdl_chain_.getNrOfJoints());
  joint_Imax.resize(kdl_chain_.getNrOfJoints());
  joint_Imin.resize(kdl_chain_.getNrOfJoints());

  J_.resize(kdl_chain_.getNrOfJoints());
  J_m_.resize(kdl_chain_.getNrOfJoints());

  transformed_force = Eigen::Vector3d::Zero();

  modelState.name.resize(kdl_chain_.getNrOfJoints());
  modelState.position.resize(kdl_chain_.getNrOfJoints());
  modelState.velocity.resize(kdl_chain_.getNrOfJoints());
  modelState.effort.resize(kdl_chain_.getNrOfJoints());

  robotState.name.resize(kdl_chain_.getNrOfJoints());
  robotState.position.resize(kdl_chain_.getNrOfJoints());
  robotState.velocity.resize(kdl_chain_.getNrOfJoints());
  robotState.effort.resize(kdl_chain_.getNrOfJoints());

  modelState.name[0] = kdl_chain_.getSegment(0).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[1] = kdl_chain_.getSegment(1).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[2] = kdl_chain_.getSegment(2).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[3] = kdl_chain_.getSegment(4).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[4] = kdl_chain_.getSegment(6).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[5] = kdl_chain_.getSegment(7).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[6] = kdl_chain_.getSegment(8).getJoint().getName(); // TODO test this stuff, better way to get joint names...

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

  qd_limit(0) = urdf_model.getJoint("l_shoulder_pan_joint"  )->limits->velocity;
  qd_limit(1) = urdf_model.getJoint("l_shoulder_lift_joint" )->limits->velocity;
  qd_limit(2) = urdf_model.getJoint("l_upper_arm_roll_joint")->limits->velocity;
  qd_limit(3) = urdf_model.getJoint("l_elbow_flex_joint"    )->limits->velocity;
  qd_limit(4) = urdf_model.getJoint("l_forearm_roll_joint"  )->limits->velocity;
  qd_limit(5) = urdf_model.getJoint("l_wrist_flex_joint"    )->limits->velocity;
  qd_limit(6) = urdf_model.getJoint("l_wrist_roll_joint"    )->limits->velocity;

  std::string nom_joint_0 = "/nom_joint_0";
  std::string nom_joint_1 = "/nom_joint_1";
  std::string nom_joint_2 = "/nom_joint_2";
  std::string nom_joint_3 = "/nom_joint_3";
  std::string nom_joint_4 = "/nom_joint_4";
  std::string nom_joint_5 = "/nom_joint_5";
  std::string nom_joint_6 = "/nom_joint_6";

  if (!n.getParam( nom_joint_0, q_nominal(0) )) { ROS_ERROR("Value not loaded from parameter: %s !)", nom_joint_0.c_str()); return false; }
  if (!n.getParam( nom_joint_1, q_nominal(1) )) { ROS_ERROR("Value not loaded from parameter: %s !)", nom_joint_1.c_str()); return false; }
  if (!n.getParam( nom_joint_2, q_nominal(2) )) { ROS_ERROR("Value not loaded from parameter: %s !)", nom_joint_2.c_str()); return false; }
  if (!n.getParam( nom_joint_3, q_nominal(3) )) { ROS_ERROR("Value not loaded from parameter: %s !)", nom_joint_3.c_str()); return false; }
  if (!n.getParam( nom_joint_4, q_nominal(4) )) { ROS_ERROR("Value not loaded from parameter: %s !)", nom_joint_4.c_str()); return false; }
  if (!n.getParam( nom_joint_5, q_nominal(5) )) { ROS_ERROR("Value not loaded from parameter: %s !)", nom_joint_5.c_str()); return false; }
  if (!n.getParam( nom_joint_6, q_nominal(6) )) { ROS_ERROR("Value not loaded from parameter: %s !)", nom_joint_6.c_str()); return false; }

  // Pick the gains.
  Kp_.vel(0) = 100.0;  Kd_.vel(0) = 1.0;        // Translation x
  Kp_.vel(1) = 100.0;  Kd_.vel(1) = 1.0;        // Translation y
  Kp_.vel(2) = 100.0;  Kd_.vel(2) = 1.0;        // Translation z
  Kp_.rot(0) = 100.0;  Kd_.rot(0) = 1.0;        // Rotation x
  Kp_.rot(1) = 100.0;  Kd_.rot(1) = 1.0;        // Rotation y
  Kp_.rot(2) = 100.0;  Kd_.rot(2) = 1.0;        // Rotation z

//  for( uint ind_ = 0; ind_ < kdl_chain_.getNrOfJoints(); ind_++ )
//  {
//	  double p,i,d,i_max,i_min;
//
//	  if (!n.getParam( "/l_arm_controller/gains/" + modelState.name[ind_] + "/p"       , p            ));
//	  if (!n.getParam( "/l_arm_controller/gains/" + modelState.name[ind_] + "/i"       , i            ));
//	  if (!n.getParam( "/l_arm_controller/gains/" + modelState.name[ind_] + "/d"       , d            ));
//	  if (!n.getParam( "/l_arm_controller/gains/" + modelState.name[ind_] + "/i_clamp" , i_max        ));
//
//	  i_min = -i_max;
//
//	  joint_P   (ind_) = p     ;
//	  joint_I   (ind_) = i     ;
//	  joint_D   (ind_) = d     ;
//	  joint_Imax(ind_) = i_max ;
//	  joint_Imin(ind_) = i_min ;
//  }

  std::string para_forceCutOffX = "/forceCutOffX";
  std::string para_forceCutOffY = "/forceCutOffY";
  std::string para_forceCutOffZ = "/forceCutOffZ";

  if (!n.getParam( para_forceCutOffX , forceCutOffX )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffX.c_str()) ; return false; }
  if (!n.getParam( para_forceCutOffY , forceCutOffY )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffY.c_str()) ; return false; }
  if (!n.getParam( para_forceCutOffZ , forceCutOffZ )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffZ.c_str()) ; return false; }

  delT = 0.001;

  /////////////////////////
  // System Model

  eigen_temp_joint.resize( num_Joints,1 );
  kdl_temp_joint_.resize(kdl_chain_.getNrOfJoints());

  q       .resize( num_Joints, 1 ) ;
  qd      .resize( num_Joints, 1 ) ;
  qdd     .resize( num_Joints, 1 ) ;
  q_m     .resize( num_Joints, 1 ) ;
  qd_m    .resize( num_Joints, 1 ) ;
  qdd_m   .resize( num_Joints, 1 ) ;

  // Cartesian states
  x_m     .resize( 6         , 1 ) ;
  xd_m    .resize( 6         , 1 ) ;
  xdd_m   .resize( 6         , 1 ) ;

  t_r     .resize( num_Joints, 1 ) ;
  task_ref.resize( num_Joints, 1 ) ;
  tau     .resize( num_Joints, 1 ) ;

  q        = Eigen::MatrixXd::Zero( num_Joints, 1 ) ;
  qd       = Eigen::MatrixXd::Zero( num_Joints, 1 ) ;
  qdd      = Eigen::MatrixXd::Zero( num_Joints, 1 ) ;
  q_m      = Eigen::MatrixXd::Zero( num_Joints, 1 ) ;
  qd_m     = Eigen::MatrixXd::Zero( num_Joints, 1 ) ;
  qdd_m    = Eigen::MatrixXd::Zero( num_Joints, 1 ) ;
  t_r      = Eigen::MatrixXd::Zero( num_Joints, 1 ) ;
  task_ref = Eigen::MatrixXd::Zero( num_Joints, 1 ) ;
  tau      = Eigen::MatrixXd::Zero( num_Joints, 1 ) ;


  // System Model END
  /////////////////////////


  /////////////////////////
  // NN

  nnController.changeNNstructure( num_Inputs  ,   // num_Inputs
                                  num_Outputs ,   // num_Outputs
                                  num_Hidden  ,   // num_Hidden
                                  num_Error   ,   // num_Error
                                  num_Joints   ); // num_Joints

  Eigen::MatrixXd p_Kv     ;
  Eigen::MatrixXd p_lambda ;

  p_Kv                  .resize( num_Joints, 1 ) ;
  p_lambda              .resize( num_Joints, 1 ) ;

  p_Kv << Kv ,
          Kv ,
          Kv ,
          Kv ,
          Kv ,
          Kv ,
          Kv ;

  p_lambda << lambda ,
              lambda ,
              lambda ,
              lambda ,
              lambda ,
              lambda ,
              lambda ;

//  p_Kv     = joint_D;
//  p_lambda = joint_P.cwiseQuotient(joint_D);

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

  if( tip_name == "l_gripper_tool_frame" )
  {
  	ft_handle_ = hardwareInterface->getForceTorque("l_gripper_motor");
  }

  if( tip_name == "r_gripper_tool_frame" )
  {
  	ft_handle_ = hardwareInterface->getForceTorque("r_gripper_motor");
  }


  if( !ft_handle_ )
      ROS_ERROR("Something wrong with getting ft handle");


//  /* get a handle to the left gripper accelerometer */
//    accelerometer_handle_ = hardwareInterface->getAccelerometer("r_gripper_motor");
//    if(!accelerometer_handle_)
//        ROS_ERROR("Something wrong with getting accelerometer handle");
//
//    // set to 1.5 kHz bandwidth (should be the default)
//    accelerometer_handle_->command_.bandwidth_ = 6;
//
//    // set to +/- 8g range (0=2g,1=4g)
//    accelerometer_handle_->command_.range_ = 2;

  pub_cycle_count_ = 0;
  should_publish_  = false;

  // Update controller paramters
  paramUpdate_srv_ = n.advertiseService("paramUpdate", &PR2NeuroadptControllerClass::paramUpdate, this);

	/////////////////////////
	// DATA COLLECTION

    set_circle_rate_srv_   = n.advertiseService("circle_rate", &PR2NeuroadptControllerClass::circleRateCB       , this);
    save_srv_              = n.advertiseService("save"       , &PR2NeuroadptControllerClass::save               , this);
    publish_srv_           = n.advertiseService("publish"    , &PR2NeuroadptControllerClass::publish            , this);
	capture_srv_           = n.advertiseService("capture"    , &PR2NeuroadptControllerClass::capture            , this);
	save_srv_              = n.advertiseService("saveData"   , &PR2NeuroadptControllerClass::saveControllerData , this);

	pubFTData_             = n.advertise< geometry_msgs::WrenchStamped             >( "FT_data"              , StoreLen);
	pubModelStates_        = n.advertise< sensor_msgs::JointState                  >( "model_joint_states"   , StoreLen);
	pubRobotStates_        = n.advertise< sensor_msgs::JointState                  >( "robot_joint_states"   , StoreLen);
	pubModelCartPos_       = n.advertise< geometry_msgs::PoseStamped               >( "model_cart_pos"       , StoreLen);
	pubRobotCartPos_       = n.advertise< geometry_msgs::PoseStamped               >( "robot_cart_pos"       , StoreLen);
	pubControllerParam_    = n.advertise< neuroadaptive_msgs::controllerParam      >( "controller_params"    , StoreLen);
	pubControllerFullData_ = n.advertise< neuroadaptive_msgs::controllerFullData   >( "controllerFullData"   , StoreLen);

	storage_index_ = StoreLen;

	// DATA COLLECTION END
	/////////////////////////

  return true;
}

/// Controller startup in realtime
void PR2NeuroadptControllerClass::starting()
{
  // Get the current joint values to compute the initial tip location.
  chain_.getPositions(q0_);
  q0_m_ = q0_;

  // Model initial conditions
  // q_m = JointKdl2Eigen(q0_m_);

  jnt_to_pose_solver_->JntToCart(q0_, x0_);
  x0_m_ = x0_;

  /////////////////////////
  // System Model


  // System Model END
  /////////////////////////


  // Initialize the phase of the circle as zero.
  circle_phase_ = 0.0;
  startCircleTraj = false;

  // Also reset the time-of-last-servo-cycle.
  last_time_ = robot_state_->getTime();
  start_time_ = robot_state_->getTime();
/*
  // set FT sensor bias due to gravity
  std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
  l_ft_samples    = l_ftData_vector.size() - 1;
  l_ftBias.wrench = l_ftData_vector[l_ft_samples];

  std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
  r_ft_samples    = r_ftData_vector.size() - 1;
  r_ftBias.wrench = r_ftData_vector[r_ft_samples];
  */
}


/// Controller update loop in realtime
void PR2NeuroadptControllerClass::update()
{

//	// retrieve our accelerometer data
//	std::vector<geometry_msgs::Vector3> threeAccs = accelerometer_handle_->state_.samples_;
//
//	threeAccs[threeAccs.size()-1].x
//	threeAccs[threeAccs.size()-1].y
//	threeAccs[threeAccs.size()-1].z

  {
	std::vector<geometry_msgs::Wrench> r_ftData_vector = ft_handle_->state_.samples_;
	r_ft_samples    = r_ftData_vector.size() - 1;
//      r_ftData.wrench = r_ftData_vector[r_ft_samples];
	ftData.wrench.force.x  =   ( r_ftData_vector[r_ft_samples].force.x  - r_ftBias.wrench.force.x  ) ;
	ftData.wrench.force.y  =   ( r_ftData_vector[r_ft_samples].force.y  - r_ftBias.wrench.force.y  ) ;
	ftData.wrench.force.z  =   ( r_ftData_vector[r_ft_samples].force.z  - r_ftBias.wrench.force.z  ) ;
	ftData.wrench.torque.x =   ( r_ftData_vector[r_ft_samples].torque.x - r_ftBias.wrench.torque.x ) ;
	ftData.wrench.torque.y =   ( r_ftData_vector[r_ft_samples].torque.y - r_ftBias.wrench.torque.y ) ;
	ftData.wrench.torque.z =   ( r_ftData_vector[r_ft_samples].torque.z - r_ftBias.wrench.torque.z ) ;
  }

  double dt;                    // Servo loop time step

  // Calculate the dt between servo cycles.
  dt = (robot_state_->getTime() - last_time_).toSec();
  last_time_ = robot_state_->getTime();

  // Get the current joint positions and velocities.
  chain_.getPositions(q_);
  chain_.getVelocities(qdot_);
  chain_.getEfforts(tau_f_);

  // Save model joints to KDL
  q_m_ = JointEigen2Kdl(q_m);

  // Compute the forward kinematics and Jacobian (at this location).
  jnt_to_pose_solver_->JntToCart(q_, x_);
  jnt_to_jac_solver_->JntToJac(q_, J_);

  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    xdot_(i) = 0;
    for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++)
      xdot_(i) += J_(i,j) * qdot_.qdot(j);
  }

  // Follow a circle of 10cm at 3 rad/sec.
  if( startCircleTraj == true )
  {
    circle_phase_ += circle_rate * dt;
  }

  KDL::Vector  circle(0,0,0);
  circle(2) = 0.1 * sin(circle_phase_);
  circle(1) = 0.1 * (cos(circle_phase_) - 1);

  xd_ = x0_;
  xd_.p += circle;

//  // Calculate a Cartesian restoring force.
//  xerr_.vel = x_.p - xd_.p;
//  xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
//                     xd_.M.UnitY() * x_.M.UnitY() +
//                     xd_.M.UnitZ() * x_.M.UnitZ());
//
//  robotCartPos_.position.x    = x_.p(0);
//  robotCartPos_.position.y    = x_.p(1);
//  robotCartPos_.position.z    = x_.p(2);
//  x_.M.GetQuaternion( robotCartPos_.orientation.x ,
//                      robotCartPos_.orientation.y ,
//                      robotCartPos_.orientation.z ,
//                      robotCartPos_.orientation.w  );
//
//  modelCartPos_.position.x    = xd_.p(0);
//  modelCartPos_.position.y    = xd_.p(1);
//  modelCartPos_.position.z    = xd_.p(2);
//  xd_.M.GetQuaternion( modelCartPos_.orientation.x ,
//                       modelCartPos_.orientation.y ,
//                       modelCartPos_.orientation.z ,
//                       modelCartPos_.orientation.w  );

//  for (unsigned int i = 0 ; i < 6 ; i++)
//  {
//    F_(i) = - Kp_(i) * xerr_(i) - Kd_(i) * xdot_(i);
//  }

  // Force control only ferr Z in ft sensor frame is x in robot frame
//  F_(0) = ferr_(2); // - Kd_(i) * xdot_(i);

  ///////////////////////////////
  // Human force input
  // Force error

  Eigen::Vector3d forceFT( ftData.wrench.force.x, ftData.wrench.force.y, ftData.wrench.force.z );

  //                               w       x       y      z
  Eigen::Quaterniond ft_to_acc(0.579, -0.406, -0.579, 0.406);
  transformed_force = ft_to_acc._transformVector( forceFT );

  if( ( transformed_force(0) < forceCutOffX ) && ( transformed_force(0) > -forceCutOffX ) ){ transformed_force(0) = 0; }
  if( ( transformed_force(1) < forceCutOffY ) && ( transformed_force(1) > -forceCutOffY ) ){ transformed_force(1) = 0; }
  if( ( transformed_force(2) < forceCutOffZ ) && ( transformed_force(2) > -forceCutOffZ ) ){ transformed_force(2) = 0; }

  transformed_force(1) = - transformed_force(1);

  // Human force input END
  ///////////////////////////////

  ferr_(0) =  transformed_force(0) ; // 30*sin(circle_phase_);
  ferr_(1) =  transformed_force(1) ; // 0				       ;
  ferr_(2) =  // transformed_force(2) ; // 0				       ;
  ferr_(3) =  0 ; // r_ftData.wrench.torque.x; // 0                    ;
  ferr_(4) =  0 ; // r_ftData.wrench.torque.y; // 0                    ;
  ferr_(5) =  0 ; // r_ftData.wrench.torque.z; // 0         s           ;

/*  // Convert the force into a set of joint torques.
  for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++)
  {
//    tau_t_(i) = 0;
    tau_h_(i) = 0;
    for (unsigned int j = 0 ; j < 6 ; j++)
    {
//      tau_t_(i) += J_(j,i) * F_(j);   // This will give the impedance to a trajectory
      tau_h_(i)+= J_(j,i) * ferr_(j); // this will give the torque from human interaction
    }
  }*/

    /////////////////////////
	// System Model

//  	// Integrator
//	tau_h_(0) = 0 ; // tau_h(0);
//	tau_h_(1) = vpol_init_x[0]; //sin(circle_phase_);    // tau_h(1);
//	tau_h_(2) = 0 ; // tau_h(2);
//	tau_h_(3) = 0 ; // tau_h(3);
//	tau_h_(4) = 0 ; // tau_h(4);
//	tau_h_(5) = 0 ; // tau_h(5);
//	tau_h_(6) = 0 ; // tau_h(6);

  // Reference torque from human interaction or trajectory following

  // Human
	t_r(0) = - tau_h_(0);
	t_r(1) =   tau_h_(1);
	t_r(2) =   tau_h_(2);
	t_r(3) =   tau_h_(3);
	t_r(4) =   tau_h_(4);
	t_r(5) =   tau_h_(5);
	t_r(6) =   tau_h_(6);

//    // Trajectory/impedance
//	t_r(0) = tau_t_(0);
//	t_r(1) = tau_t_(1);
//	t_r(2) = tau_t_(2);
//	t_r(3) = tau_t_(3);
//	t_r(4) = tau_t_(4);
//	t_r(5) = tau_t_(5);
//	t_r(6) = tau_t_(6);

	// Current joint positions and velocities
	q = JointKdl2Eigen( q_ );
	qd = JointVelKdl2Eigen( qdot_ );


//	0 - 'r_upper_arm_roll_joint' || "r_shoulder_pan_joint"   -0.309261  : 18 : r_shoulder_pan_joint
//	1 - 'r_shoulder_pan_joint'    | "r_shoulder_lift_joint"  -0.0340454 : 19 : r_shoulder_lift_joint
//	2 - 'r_shoulder_lift_joint'   | "r_upper_arm_roll_joint" -1.55365   : 17 : r_upper_arm_roll_joint
//	3 - 'r_forearm_roll_joint'   || "r_elbow_flex_joint"     -1.84681   : 21 : r_elbow_flex_joint
//	4 - 'r_elbow_flex_joint'      | "r_forearm_roll_joint"   -1.57489   : 20 : r_forearm_roll_joint
//	5 - 'r_wrist_flex_joint'      | "r_wrist_flex_joint"     -1.54739   : 22 : r_wrist_flex_joint
//	6 - 'r_wrist_roll_joint'      | "r_wrist_roll_joint"     -0.0322204 : 23 : r_wrist_roll_joint


	if( (robot_state_->getTime() - start_time_).toSec() > 5 )
        {
	  task_ref   (0) = -0.821127 ;
	  task_ref   (3) = -1.70016  ;
        }else
        {
          start_time_ = robot_state_->getTime();
          task_ref   (0) = -0.190788 ;
          task_ref   (3) = -1.42669  ;
        }

	q_m(0) = q_nominal(0) ;  qd_m(0) = 0;  qdd_m(0) = 0;
	q_m(1) = q_nominal(1) ;  qd_m(1) = 0;  qdd_m(1) = 0;
	q_m(2) = q_nominal(2) ;  qd_m(2) = 0;  qdd_m(2) = 0;
	q_m(3) = q_nominal(3) ;  qd_m(3) = 0;  qdd_m(3) = 0;
	q_m(4) = q_nominal(4) ;  qd_m(4) = 0;  qdd_m(4) = 0;
	q_m(5) = q_nominal(5) ;  qd_m(5) = 0;  qdd_m(5) = 0;
	q_m(6) = q_nominal(6) ;  qd_m(6) = 0;  qdd_m(6) = 0;


        x_m(2) = 0.03 ;
        x_m(3) = 0 ;
        x_m(4) = 0 ;
        x_m(5) = 0 ;

/*        // Compute the forward kinematics and Jacobian of the model (at this location).
        jnt_to_pose_solver_->JntToCart(q_m_, x_m_);

        modelCartPos_.position.x    = x_m_.p(0);
        modelCartPos_.position.y    = x_m_.p(1);
        modelCartPos_.position.z    = x_m_.p(2);
        x_m_.M.GetQuaternion( modelCartPos_.orientation.x ,
                            modelCartPos_.orientation.y ,
                            modelCartPos_.orientation.z ,
                            modelCartPos_.orientation.w  );*/

        xd_m(2) = 0          ;
        xd_m(3) = xdot_  (3) ;
        xd_m(4) = xdot_  (4) ;
        xd_m(5) = xdot_  (5) ;

        xdd_m(2) = 0 ;
        xdd_m(3) = 0 ;
        xdd_m(4) = 0 ;
        xdd_m(5) = 0 ;

        kdl_xd_m_(0) = xd_m(0);
        kdl_xd_m_(1) = xd_m(1);
        kdl_xd_m_(2) = xd_m(2);
        kdl_xd_m_(3) = xd_m(3);
        kdl_xd_m_(4) = xd_m(4);
        kdl_xd_m_(5) = xd_m(5);

	// System Model END
	/////////////////////////

        double circleAmpl = (circleUlim - circleLlim)/2 ;

	// DEBUG
//	q_m(0)  =   0 ; //- 0.5 * (sin(circle_phase_) + 1 );
//	q_m(1)  =   0 ; //- 0.5 * (sin(circle_phase_) + 1 );
//	q_m(2)  =   0 ; //- 0.5 * (sin(circle_phase_) + 1 );
	q_m(3)  = - ( circleAmpl * sin(circle_phase_) + circleAmpl + circleLlim );
//	q_m(4)  =   0;
//	q_m(5)  =   0;
//	q_m(6)  =   0;

//	qd_m(0) =   0;
//	qd_m(1) =   0;
//	qd_m(2) =   0;
	qd_m(3) = - circleAmpl * (cos(circle_phase_));
//	qd_m(4) =   0;
//	qd_m(5) =   0;
//	qd_m(6) =   0;



  /////////////////////////

  // NN
  nnController.UpdateJoint( q     ,
                            qd    ,
                            q_m   ,
                            qd_m  ,
                            qdd_m ,
                            t_r   ,
                            tau    );
  // NN END
  /////////////////////////

	// Convert from Eigen to KDL
//	tau_c_ = JointEigen2Kdl( tau );

	tau_c_(0) = tau(0);
	tau_c_(1) = tau(1);
	tau_c_(2) = tau(2);
	tau_c_(3) = tau(3);
	tau_c_(4) = tau(4);
	tau_c_(5) = tau(5);
	tau_c_(6) = tau(6);

	modelState.header.stamp = robot_state_->getTime();
	robotState.header.stamp = robot_state_->getTime();

	modelState.position[0] = q_m(0);
	modelState.position[1] = q_m(1);
	modelState.position[2] = q_m(2);
	modelState.position[3] = q_m(3);
	modelState.position[4] = q_m(4);
	modelState.position[5] = q_m(5);
	modelState.position[6] = q_m(6);

	modelState.velocity[0] = qd_m(0); // ode_init_x[0 ] ; // tau(0);
	modelState.velocity[1] = qd_m(1); // ode_init_x[1 ] ; // tau(1);
	modelState.velocity[2] = qd_m(2); // ode_init_x[2 ] ; // tau(2);
	modelState.velocity[3] = qd_m(3); // ode_init_x[3 ] ; // tau(3);
	modelState.velocity[4] = qd_m(4); // ode_init_x[4 ] ; // tau(4);
	modelState.velocity[5] = qd_m(5); // ode_init_x[5 ] ; // tau(5);
	modelState.velocity[6] = qd_m(6); // ode_init_x[6 ] ; // tau(6);

	// Input torque to mode | torque from human
	modelState.effort[0] = t_r(0);
	modelState.effort[1] = t_r(1);
	modelState.effort[2] = t_r(2);
	modelState.effort[3] = t_r(3);
	modelState.effort[4] = t_r(4);
	modelState.effort[5] = t_r(5);
	modelState.effort[6] = t_r(6);

	robotState.position[0] = q(0);
	robotState.position[1] = q(1);
	robotState.position[2] = q(2);
	robotState.position[3] = q(3);
	robotState.position[4] = q(4);
	robotState.position[5] = q(5);
	robotState.position[6] = q(6);

	robotState.velocity[0] = qd(0);
	robotState.velocity[1] = qd(1);
	robotState.velocity[2] = qd(2);
	robotState.velocity[3] = qd(3);
	robotState.velocity[4] = qd(4);
	robotState.velocity[5] = qd(5);
	robotState.velocity[6] = qd(6);

	// Output torque from controller that is sent to the robot
	robotState.effort[0] = tau_c_(0);
	robotState.effort[1] = tau_c_(1);
	robotState.effort[2] = tau_c_(2);
	robotState.effort[3] = tau_c_(3);
	robotState.effort[4] = tau_c_(4);
	robotState.effort[5] = tau_c_(5);
	robotState.effort[6] = tau_c_(6);

	// And finally send these torques out.
        chain_.setEfforts(tau_c_);

	/////////////////////////
	// DATA COLLECTION

        bufferData( dt );

	// DATA COLLECTION END
	/////////////////////////

}


void PR2NeuroadptControllerClass::bufferData( double & dt )
{
	int index = storage_index_;
	if ((index >= 0) && (index < StoreLen))
	{
		msgControllerFullData[index].dt                = dt                          ;

        // Force Data
        msgControllerFullData[index].force_x           = transformed_force(0)        ; // r_ftData.wrench.force.x     ;
        msgControllerFullData[index].force_y           = transformed_force(1)        ; // r_ftData.wrench.force.y     ;
        msgControllerFullData[index].force_z           = transformed_force(2)        ; // r_ftData.wrench.force.z     ;
        msgControllerFullData[index].torque_x          = 0                           ; // r_ftData.wrench.torque.x    ;
        msgControllerFullData[index].torque_y          = 0                           ; // r_ftData.wrench.torque.y    ;
        msgControllerFullData[index].torque_z          = 0                           ; // r_ftData.wrench.torque.z    ;

		// Input reference efforts(torques)
		msgControllerFullData[index].reference_eff_j0  = t_r(0)                      ;
		msgControllerFullData[index].reference_eff_j1  = t_r(1)                      ;
		msgControllerFullData[index].reference_eff_j2  = t_r(2)                      ;
		msgControllerFullData[index].reference_eff_j3  = t_r(3)                      ;
		msgControllerFullData[index].reference_eff_j4  = t_r(4)                      ;
		msgControllerFullData[index].reference_eff_j5  = t_r(5)                      ;
		msgControllerFullData[index].reference_eff_j6  = t_r(6)                      ;

		// Model States
		msgControllerFullData[index].m_cartPos_x       = modelCartPos_.position.x    ;
		msgControllerFullData[index].m_cartPos_y       = modelCartPos_.position.y    ;
		msgControllerFullData[index].m_cartPos_z       = modelCartPos_.position.z    ;
		msgControllerFullData[index].m_cartPos_Qx      = modelCartPos_.orientation.x ;
		msgControllerFullData[index].m_cartPos_Qy      = modelCartPos_.orientation.y ;
		msgControllerFullData[index].m_cartPos_Qz      = modelCartPos_.orientation.z ;
		msgControllerFullData[index].m_cartPos_QW      = modelCartPos_.orientation.w ;

		msgControllerFullData[index].m_pos_x           = x_m(0)                      ;
		msgControllerFullData[index].m_pos_y           = x_m(1)                      ;
		msgControllerFullData[index].m_pos_z           = x_m(2)                      ;

		msgControllerFullData[index].m_vel_x           = xd_m(0)                     ;
        msgControllerFullData[index].m_vel_y           = xd_m(1)                     ;
        msgControllerFullData[index].m_vel_z           = xd_m(2)                     ;

        msgControllerFullData[index].m_acc_x           = xdd_m(0)                    ;
        msgControllerFullData[index].m_acc_y           = xdd_m(1)                    ;
        msgControllerFullData[index].m_acc_z           = xdd_m(2)                    ;

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

		// TODO fix this
		// Model Params
		msgControllerFullData[index].m                 =   0 ; // outerLoopMSDmodel.getMass(  )(0,0) ;
		msgControllerFullData[index].d                 =   0 ; // outerLoopMSDmodel.getSpring()(0,0) ;
		msgControllerFullData[index].k                 =   0 ; // outerLoopMSDmodel.getDamper()(0,0) ;

		// Increment for the next cycle.
		storage_index_ = index+1;
	}
}

/// Service call to capture and extract the data
bool PR2NeuroadptControllerClass::paramUpdate( neuroadaptive_msgs::controllerParamUpdate::Request  & req ,
                                               neuroadaptive_msgs::controllerParamUpdate::Response & resp )
{

//  req.msg.m                ;
//  req.msg.d                ;
//  req.msg.k                ;

  num_Inputs  = req.msg.inParams         ;
  num_Outputs = req.msg.outParams        ;
  num_Hidden  = req.msg.hiddenNodes      ;
  num_Error   = req.msg.errorParams      ;

  kappa       = req.msg.kappa            ;
  Kv          = req.msg.Kv               ;
  lambda      = req.msg.lambda           ;
  Kz          = req.msg.Kz               ;
  Zb          = req.msg.Zb               ;
  fFForce     = req.msg.feedForwardForce ;
  nn_ON       = req.msg.nn_ON            ;
  nnF         = req.msg.F                ;
  nnG         = req.msg.G                ;

  nnController.changeNNstructure( num_Inputs  ,   // num_Inputs
                                  num_Outputs ,   // num_Outputs
                                  num_Hidden  ,   // num_Hidden
                                  num_Error   ,   // num_Error
                                  num_Joints   ); // num_Joints

  Eigen::MatrixXd p_Kv     ;
  Eigen::MatrixXd p_lambda ;

  p_Kv                  .resize( num_Joints, 1 ) ;
  p_lambda              .resize( num_Joints, 1 ) ;

  p_Kv << Kv ,
          Kv ,
          Kv ,
          Kv ,
          Kv ,
          Kv ,
          Kv ;

  p_lambda << lambda ,
              lambda ,
              lambda ,
              lambda ,
              lambda ,
              lambda ,
              lambda ;

  nnController.init( kappa  ,
                     p_Kv     ,
                     p_lambda ,
                     Kz     ,
                     Zb     ,
                     fFForce,
                     nnF    ,
                     nnG    ,
                     nn_ON   );

  resp.success = true;

  return true;
}

/// Service call to save the data
bool PR2NeuroadptControllerClass::circleRateCB( neuroadaptive_msgs::setCircleRate::Request & req,
		                                        neuroadaptive_msgs::setCircleRate::Response& resp )
{

  circle_rate = req.circle_rate;
  resp.success = true;

  return true;
}

/// Service call to save the data
bool PR2NeuroadptControllerClass::save( neuroadaptive_msgs::saveControllerData::Request & req,
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
//  saveDataFile.open(fileNameProto.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);

  resp.success = true;

  return true;
}

/// Service call to publish the saved data
bool PR2NeuroadptControllerClass::publish( std_srvs::Empty::Request & req,
                                           std_srvs::Empty::Response& resp )
{
  /* Then we can publish the buffer contents. */
  int  index;
  for (index = 0 ; index < StoreLen ; index++)
  {
      pubControllerFullData_.publish(msgControllerFullData[index]);
  }

  return true;
}

/// Service call to capture and extract the data
bool PR2NeuroadptControllerClass::capture( std_srvs::Empty::Request& req,
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

  // Stop circle traj
  circle_phase_   = 0.0;
  startCircleTraj = false;

  /* Then we can publish the buffer contents. */
  int  index;
  for (index = 0 ; index < StoreLen ; index++)
  {
	  pubControllerFullData_.publish(msgControllerFullData[index]);
  }

  return true;
}

/// Service call to capture and extract the data
bool PR2NeuroadptControllerClass::saveControllerData( neuroadaptive_msgs::saveControllerData::Request&  req,
                                                      neuroadaptive_msgs::saveControllerData::Response& resp )
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

  // Stop circle traj
  circle_phase_   = 0.0;
  startCircleTraj = false;

//  /* Then we can publish the buffer contents. */
//  int  index;
//  for (index = 0 ; index < StoreLen ; index++)
//  {
//          msgControllerFullData[index];
//  }

  return true;

}

Eigen::MatrixXd
PR2NeuroadptControllerClass::JointKdl2Eigen( KDL::JntArray & joint_ )
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
PR2NeuroadptControllerClass::JointVelKdl2Eigen( KDL::JntArrayVel & joint_ )
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
PR2NeuroadptControllerClass::JointEigen2Kdl( Eigen::VectorXd & joint )
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

/// Controller stopping in realtime
void PR2NeuroadptControllerClass::stopping()
{

}


// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS( PR2NeuroadptControllerClass,
		          	      pr2_controller_ns::PR2NeuroadptControllerClass,
                          pr2_controller_interface::Controller )
