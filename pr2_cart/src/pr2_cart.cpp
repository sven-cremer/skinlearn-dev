/***********************************************************************************************************************
FILENAME:    pr2_cart.cpp
AUTHORS:     Sven Cremer 		sven.cremer@mavs.uta.edu
             Isura Ranatunga	isura.ranatunga@mavs.uta.edu
			 University of Texas at Arlington, Copyright (C) 2013.

DESCRIPTION:
Realtime impedance and velocity controller for PR2 cart behavior

REFERENCES:
-This is a modified version of Isura's uta_pr2_cartpush package.
-"Writing a realtime Cartesian controller" ROS tutorial:
http://wiki.ros.org/pr2_mechanism/Tutorials/Writing%20a%20realtime%20Cartesian%20controller

PUBLISHES:  NA
SUBSCRIBES: NA
SERVICES:   NA

REVISION HISTORY:
2014.02.07  SC     original file creation
2014.02.14  SC     code cleanup
2014.02.17  SC     capturing data from a controller
2015.10.26  SC     ported to hydro
***********************************************************************************************************************/

#include "pr2_cart/pr2_cart.h"
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;

/***********************************************************************************************************************
Controller initialization in non-realtime
***********************************************************************************************************************/

bool PR2CartClass::init( pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n )
{
  // Get the root and tip link names from parameter server.
  std::string root_name, r_tip_name, l_tip_name;
  if (!n.getParam("root_name", root_name))
  {
    ROS_ERROR("No root name given in namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("r_tip_name", r_tip_name))
  {
    ROS_ERROR("No tip name given in namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("l_tip_name", l_tip_name))
  {
    ROS_ERROR("No tip name given in namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  // Construct a chain from the root to the tip and prepare the kinematics.
  // Note the joints must be calibrated.
  if (!r_chain_.init(robot, root_name, r_tip_name))
  {
    ROS_ERROR("MyCartController could not use the chain from '%s' to '%s'",
              root_name.c_str(), r_tip_name.c_str());
    return false;
  }
  // Construct a chain from the root to the tip and prepare the kinematics.
  // Note the joints must be calibrated.
  if (!l_chain_.init(robot, root_name, l_tip_name))
  {
    ROS_ERROR("MyCartController could not use the chain from '%s' to '%s'",
              root_name.c_str(), l_tip_name.c_str());
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
  }
  ROS_INFO("Successfully parsed URDF file");

  /***********************************************
   Load parameters from YAML file
   ***********************************************/

  // Velocity controller
  std::string para_P_velGain = "/velPGain" ;
  std::string para_D_velGain = "/velDGain" ;

  std::string para_P_rotGain = "/rotPGain" ;
  std::string para_D_rotGain = "/rotDGain" ;

  std::string para_rThresh   = "/rThresh"   ;
  std::string para_psiThresh = "/psiThresh" ;

  if (!n.getParam( para_P_velGain , velPGain ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", para_P_velGain.c_str()) ; return false; }
  if (!n.getParam( para_D_velGain , velDGain ))
 { ROS_ERROR("Value not loaded from parameter: %s !)", para_D_velGain.c_str()) ; return false; }

  if (!n.getParam( para_P_rotGain , rotPGain ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", para_P_rotGain.c_str()) ; return false; }
  if (!n.getParam( para_D_rotGain , rotDGain ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", para_D_rotGain.c_str()) ; return false; }

  if (!n.getParam( para_rThresh , rThresh ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", para_rThresh.c_str()) ; return false; }
  if (!n.getParam( para_psiThresh , psiThresh ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", para_psiThresh.c_str()) ; return false; }


  // Torque controller
  std::string para_P_restGain = "/restPGain" ;
  std::string para_D_restGain = "/restDGain" ;

  if (!n.getParam( para_P_restGain , restPGain ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", para_P_restGain.c_str()) ; return false; }
  if (!n.getParam( para_D_restGain , restDGain ))
 { ROS_ERROR("Value not loaded from parameter: %s !)", para_D_restGain.c_str()) ; return false; }


  // Impedance controller
  std::string para_Kp_vel_x = "/Kp_vel_x" ;
  std::string para_Kp_vel_y = "/Kp_vel_y" ;
  std::string para_Kp_vel_z = "/Kp_vel_z" ;
  std::string para_Kd_vel_x = "/Kd_vel_x" ;
  std::string para_Kd_vel_y = "/Kd_vel_y" ;
  std::string para_Kd_vel_z = "/Kd_vel_z" ;

  std::string para_Kp_rot_x = "/Kp_rot_x" ;
  std::string para_Kp_rot_y = "/Kp_rot_y" ;
  std::string para_Kp_rot_z = "/Kp_rot_z" ;
  std::string para_Kd_rot_x = "/Kd_rot_x" ;
  std::string para_Kd_rot_y = "/Kd_rot_y" ;
  std::string para_Kd_rot_z = "/Kd_rot_z" ;

  if (!n.getParam( para_Kp_vel_x , Kp_.vel(0) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kp_vel_x.c_str()) ; return false; }
  if (!n.getParam( para_Kp_vel_y , Kp_.vel(1) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kp_vel_y.c_str()) ; return false; }
  if (!n.getParam( para_Kp_vel_z , Kp_.vel(2) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kp_vel_z.c_str()) ; return false; }

  if (!n.getParam( para_Kd_vel_x , Kd_.vel(0) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kd_vel_x.c_str()) ; return false; }
  if (!n.getParam( para_Kd_vel_y , Kd_.vel(1) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kd_vel_y.c_str()) ; return false; }
  if (!n.getParam( para_Kd_vel_z , Kd_.vel(2) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kd_vel_z.c_str()) ; return false; }

  if (!n.getParam( para_Kp_rot_x , Kp_.rot(0) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kp_rot_x.c_str()) ; return false; }
  if (!n.getParam( para_Kp_rot_y , Kp_.rot(1) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kp_rot_y.c_str()) ; return false; }
  if (!n.getParam( para_Kp_rot_z , Kp_.rot(2) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kp_rot_z.c_str()) ; return false; }

  if (!n.getParam( para_Kd_rot_x , Kd_.rot(0) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kd_rot_x.c_str()) ; return false; }
  if (!n.getParam( para_Kd_rot_y , Kd_.rot(1) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kd_rot_y.c_str()) ; return false; }
  if (!n.getParam( para_Kd_rot_z , Kd_.rot(2) )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_Kd_rot_z.c_str()) ; return false; }



  // Store the robot handle for later use (to get time).
  robot_state_ = robot;

  // Construct the kdl solvers in non-realtime.
  r_chain_.toKDL(r_kdl_chain_);
  l_chain_.toKDL(l_kdl_chain_);
  r_jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(r_kdl_chain_));
  r_jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(        r_kdl_chain_));
  l_jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(l_kdl_chain_));
  l_jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(        l_kdl_chain_));

  // Resize (pre-allocate) the variables in non-realtime.
  r_q_.resize(    r_kdl_chain_.getNrOfJoints());
  r_q0_.resize(   r_kdl_chain_.getNrOfJoints());
  r_qdot_.resize( r_kdl_chain_.getNrOfJoints());
  r_tau_.resize(  r_kdl_chain_.getNrOfJoints());
  tau_h.resize(   r_kdl_chain_.getNrOfJoints());

  q_m_.resize(    r_kdl_chain_.getNrOfJoints());
  qd_m_.resize(   r_kdl_chain_.getNrOfJoints());
  qdd_m_.resize(  r_kdl_chain_.getNrOfJoints());

  q_lower.resize( r_kdl_chain_.getNrOfJoints());
  q_upper.resize( r_kdl_chain_.getNrOfJoints());
  qd_limit.resize(r_kdl_chain_.getNrOfJoints());

  r_J_.resize(    r_kdl_chain_.getNrOfJoints());

  l_q_.resize(    l_kdl_chain_.getNrOfJoints());
  l_q0_.resize(   l_kdl_chain_.getNrOfJoints());
  l_qdot_.resize( l_kdl_chain_.getNrOfJoints());
  l_tau_.resize(  l_kdl_chain_.getNrOfJoints());
//  tau_h.resize(   l_kdl_chain_.getNrOfJoints());
//
//  q_m_.resize(    l_kdl_chain_.getNrOfJoints());
//  qd_m_.resize(   l_kdl_chain_.getNrOfJoints());
//  qdd_m_.resize(  l_kdl_chain_.getNrOfJoints());
//
//  q_lower.resize( l_kdl_chain_.getNrOfJoints());
//  q_upper.resize( l_kdl_chain_.getNrOfJoints());
//  qd_limit.resize(l_kdl_chain_.getNrOfJoints());

  l_J_.resize(    l_kdl_chain_.getNrOfJoints());

  q_lower(0) = urdf_model.getJoint("r_shoulder_pan_joint"  )->limits->lower;
  q_lower(1) = urdf_model.getJoint("r_shoulder_lift_joint" )->limits->lower;
  q_lower(2) = urdf_model.getJoint("r_upper_arm_roll_joint")->limits->lower;
  q_lower(3) = urdf_model.getJoint("r_elbow_flex_joint"    )->limits->lower;
  q_lower(4) = urdf_model.getJoint("r_forearm_roll_joint"  )->limits->lower;
  q_lower(5) = urdf_model.getJoint("r_wrist_flex_joint"    )->limits->lower;
  q_lower(6) = urdf_model.getJoint("r_wrist_roll_joint"    )->limits->lower;

  q_upper(0) = urdf_model.getJoint("r_shoulder_pan_joint"  )->limits->upper;
  q_upper(1) = urdf_model.getJoint("r_shoulder_lift_joint" )->limits->upper;
  q_upper(2) = urdf_model.getJoint("r_upper_arm_roll_joint")->limits->upper;
  q_upper(3) = urdf_model.getJoint("r_elbow_flex_joint"    )->limits->upper;
  q_upper(4) = urdf_model.getJoint("r_forearm_roll_joint"  )->limits->upper;
  q_upper(5) = urdf_model.getJoint("r_wrist_flex_joint"    )->limits->upper;
  q_upper(6) = urdf_model.getJoint("r_wrist_roll_joint"    )->limits->upper;

  qd_limit(0) = urdf_model.getJoint("r_shoulder_pan_joint"  )->limits->velocity;
  qd_limit(1) = urdf_model.getJoint("r_shoulder_lift_joint" )->limits->velocity;
  qd_limit(2) = urdf_model.getJoint("r_upper_arm_roll_joint")->limits->velocity;
  qd_limit(3) = urdf_model.getJoint("r_elbow_flex_joint"    )->limits->velocity;
  qd_limit(4) = urdf_model.getJoint("r_forearm_roll_joint"  )->limits->velocity;
  qd_limit(5) = urdf_model.getJoint("r_wrist_flex_joint"    )->limits->velocity;
  qd_limit(6) = urdf_model.getJoint("r_wrist_roll_joint"    )->limits->velocity;


  controller_on = true;
  forceTorque_on = false;

  /* get a handle to the hardware interface */
  pr2_hardware_interface::HardwareInterface* hardwareInterface = robot->model_->hw_;
  if(!hardwareInterface)
      ROS_ERROR("Something wrong with the hardware interface pointer!");

  if(forceTorque_on)
  {
	  l_ft_handle_ = hardwareInterface->getForceTorque("l_gripper_motor");
	  r_ft_handle_ = hardwareInterface->getForceTorque("r_gripper_motor");

	  if( !l_ft_handle_ /*wristFTdata.getLeftHandle()*/ )
		  ROS_ERROR("Something wrong with getting l_ft handle");
	  if( !r_ft_handle_ /*wristFTdata.getRightHandle()*/ )
		  ROS_ERROR("Something wrong with getting r_ft handle");
  }
  pub_cycle_count_ = 0;
  should_publish_  = false;

  // Initialize realtime publisher to publish to ROS topic
  pubBaseMove_.init(node,"base_controller/command", 1);


  // Services for changing gains
  srv_velPGain 	= n.advertiseService("set_velPGain", 	&PR2CartClass::set_velPGain, this);
  srv_velDGain 	= n.advertiseService("set_velDGain", 	&PR2CartClass::set_velDGain, this);
  srv_rotPGain 	= n.advertiseService("set_rotPGain", 	&PR2CartClass::set_rotPGain, this);
  srv_rotDGain 	= n.advertiseService("set_rotDGain", 	&PR2CartClass::set_rotDGain, this);
  srv_rThresh 	= n.advertiseService("set_rThresh", 	&PR2CartClass::set_rThresh, this);
  srv_psiThresh = n.advertiseService("set_psiThresh", 	&PR2CartClass::set_psiThresh, this);

  srv_restPGain = n.advertiseService("set_restPGain", 	&PR2CartClass::set_restPGain, this);
  srv_restDGain = n.advertiseService("set_restDGain", 	&PR2CartClass::set_restDGain, this);

  srv_Kp_vel 	= n.advertiseService("set_Kp_vel", 		&PR2CartClass::set_Kp_vel, this);
  srv_Kd_vel 	= n.advertiseService("set_Kd_vel", 		&PR2CartClass::set_Kd_vel, this);
  srv_Kp_rot 	= n.advertiseService("set_Kp_rot", 		&PR2CartClass::set_Kp_rot, this);
  srv_Kd_rot 	= n.advertiseService("set_Kd_rot", 		&PR2CartClass::set_Kd_rot, this);

  srv_getState = n.advertiseService("getState", 		&PR2CartClass::get_State, this);


  // Capturing data from controller
   capture_srv_ = n.advertiseService("capture", &PR2CartClass::capture, this);
   capture_pub_ = n.advertise<ice_msgs::combinedTwistError>("controllerState_topic", StoreLen);
   storage_index_ = StoreLen;

   // Init realtime publishers for errors
   realtime_pub_pose_error.init(node,"/pr2_cartPull/combined_pose_error",1);
   realtime_pub_twist_error.init(node,"/pr2_cartPull/combined_twist_error",1);


  for(int i = 0;i<10;i++)
  {
	  l_force_buffer[i] = 0;
	  r_force_buffer[i] = 0;
  }
  l_buffer_indx = 0;
  r_buffer_indx = 0;

  return true;
}


/***********************************************************************************************************************
Service call to set gains
***********************************************************************************************************************/
// Velocity controller
bool PR2CartClass::set_velPGain(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp)
{
  velPGain = req.value;
  ROS_INFO("PR2CartPull: set velPGain to %f", req.value);
  resp.value = velPGain;
  return true;
}
bool PR2CartClass::set_velDGain(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp)
{
  velDGain = req.value;
  ROS_INFO("PR2CartPull: set velDGain to %f", req.value);
  resp.value = velDGain;
  return true;
}
bool PR2CartClass::set_rotPGain(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp)
{
  rotPGain = req.value;
  ROS_INFO("PR2CartPull: set rotPGain to %f", req.value);
  resp.value = rotPGain;
  return true;
}
bool PR2CartClass::set_rotDGain(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp)
{
  rotDGain = req.value;
  ROS_INFO("PR2CartPull: set rotDGain to %f", req.value);
  resp.value = rotDGain;
  return true;
}
bool PR2CartClass::set_rThresh(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp)
{
  rThresh = req.value;
  ROS_INFO("PR2CartPull: set rotDGain to %f", req.value);
  resp.value = rThresh;
  return true;
}
bool PR2CartClass::set_psiThresh(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp)
{
  psiThresh = req.value;
  ROS_INFO("PR2CartPull: set rotDGain to %f", req.value);
  resp.value = psiThresh;
  return true;
}
// Torque controller
bool PR2CartClass::set_restPGain(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp)
{
  restPGain = req.value;
  ROS_INFO("PR2CartPull: set rotPGain to %f", req.value);
  resp.value = restPGain;
  return true;
}
bool PR2CartClass::set_restDGain(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp)
{
  restDGain = req.value;
  ROS_INFO("PR2CartPull: set rotDGain to %f", req.value);
  resp.value = restDGain;
  return true;
}

// Impedance controller
bool PR2CartClass::set_Kp_vel(ice_msgs::setGains::Request& req, ice_msgs::setGains::Response& resp)
{
	Kp_.vel(0) = req.K_x;        // Translation x
	Kp_.vel(1) = req.K_y;        // Translation y
	Kp_.vel(2) = req.K_z;        // Translation z

	ROS_INFO("PR2CartPull: set Kp_vel to [%f %f %f]", req.K_x, req.K_y, req.K_z);
	resp.completed = true;
	return true;
}
bool PR2CartClass::set_Kd_vel(ice_msgs::setGains::Request& req, ice_msgs::setGains::Response& resp)
{
	Kd_.vel(0) = req.K_x;        // Translation x
	Kd_.vel(1) = req.K_y;        // Translation y
	Kd_.vel(2) = req.K_z;        // Translation z

	ROS_INFO("PR2CartPull: set Kd_vel to [%f %f %f]", req.K_x, req.K_y, req.K_z);
	resp.completed = true;
	return true;
}
bool PR2CartClass::set_Kp_rot(ice_msgs::setGains::Request& req, ice_msgs::setGains::Response& resp)
{
	Kp_.rot(0) = req.K_x;        // Translation x
	Kp_.rot(1) = req.K_y;        // Translation y
	Kp_.rot(2) = req.K_z;        // Translation z

	ROS_INFO("PR2CartPull: set Kp_rot to [%f %f %f]", req.K_x, req.K_y, req.K_z);
	resp.completed = true;
	return true;
}
bool PR2CartClass::set_Kd_rot(ice_msgs::setGains::Request& req, ice_msgs::setGains::Response& resp)
{
	Kd_.rot(0) = req.K_x;        // Translation x
	Kd_.rot(1) = req.K_y;        // Translation y
	Kd_.rot(2) = req.K_z;        // Translation z

	ROS_INFO("PR2CartPull: set Kd_rot to [%f %f %f]", req.K_x, req.K_y, req.K_z);
	resp.completed = true;
	return true;
}
/***********************************************************************************************************************
Service call to get current gain and threshold values
***********************************************************************************************************************/
bool PR2CartClass::get_State(ice_msgs::getState::Request& req, ice_msgs::getState::Response& resp)
{
	resp.restPGain	 = restPGain;
	resp.restDGain	 = restDGain;
	resp.velPGain	 = velPGain;
	resp.velDGain	 = velDGain;
	resp.rotPGain	 = rotPGain;
	resp.rotDGain	 = rotDGain;
	resp.rThresh	 = rThresh;
	resp.psiThresh	 = psiThresh;
	resp.Kp_vel_x	 = Kp_.vel(0);
	resp.Kp_vel_y	 = Kp_.vel(1);
	resp.Kp_vel_z	 = Kp_.vel(2);
	resp.Kd_vel_x	 = Kd_.vel(0);
	resp.Kd_vel_y	 = Kd_.vel(1);
	resp.Kd_vel_z	 = Kd_.vel(2);
	resp.Kp_rot_x	 = Kp_.rot(0);
	resp.Kp_rot_y	 = Kp_.rot(1);
	resp.Kp_rot_z	 = Kp_.rot(2);
	resp.Kd_rot_x	 = Kd_.rot(0);
	resp.Kd_rot_y	 = Kd_.rot(1);
	resp.Kd_rot_z	 = Kd_.rot(2);
	return true;
}

/***********************************************************************************************************************
Controller startup in realtime - sets initial arm position (if removed the PR2 goes crazy)
***********************************************************************************************************************/
void PR2CartClass::starting()
{

	// Get the current joint values to compute the initial tip location.
	r_chain_.getPositions(r_q0_);
	r_jnt_to_pose_solver_->JntToCart(r_q0_, r_x0_);

	l_chain_.getPositions(l_q0_);
	l_jnt_to_pose_solver_->JntToCart(l_q0_, l_x0_);

	// Also reset the time-of-last-servo-cycle.
	last_time_ = robot_state_->getTime();

	if( forceTorque_on )
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

/***********************************************************************************************************************
Controller update loop in realtime
***********************************************************************************************************************/
void PR2CartClass::update()
{
  if (forceTorque_on)
  {
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
	r_ftData.wrench.force.x  = r_ftData_vector[r_ft_samples].force.x  - r_ftBias.wrench.force.x ;
	r_ftData.wrench.force.y  = r_ftData_vector[r_ft_samples].force.y  - r_ftBias.wrench.force.y ;
	r_ftData.wrench.force.z  = r_ftData_vector[r_ft_samples].force.z  - r_ftBias.wrench.force.z ;
	r_ftData.wrench.torque.x = r_ftData_vector[r_ft_samples].torque.x - r_ftBias.wrench.torque.x;
	r_ftData.wrench.torque.y = r_ftData_vector[r_ft_samples].torque.y - r_ftBias.wrench.torque.y;
	r_ftData.wrench.torque.z = r_ftData_vector[r_ft_samples].torque.z - r_ftBias.wrench.torque.z;
  }

  // Calculate the dt between servo cycles.
  dt = (robot_state_->getTime() - last_time_).toSec();
  last_time_ = robot_state_->getTime();

  // Get the current joint positions and velocities.
  r_chain_.getPositions( r_q_);
  r_chain_.getVelocities(r_qdot_);

  l_chain_.getPositions( l_q_);
  l_chain_.getVelocities(l_qdot_);

  // Compute the forward kinematics and Jacobian (at this location).
  r_jnt_to_pose_solver_->JntToCart(r_q_, r_x_);
  r_jnt_to_jac_solver_->JntToJac(  r_q_, r_J_);

  l_jnt_to_pose_solver_->JntToCart(l_q_, l_x_);
  l_jnt_to_jac_solver_->JntToJac(  l_q_, l_J_);

  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    r_xdot_(i) = 0;
    l_xdot_(i) = 0;
    for (unsigned int j = 0 ; j < r_kdl_chain_.getNrOfJoints() ; j++)
    {
      r_xdot_(i) += r_J_(i,j) * r_qdot_.qdot(j);
      l_xdot_(i) += l_J_(i,j) * l_qdot_.qdot(j);
    }
  }


  r_xd_    = r_x0_ ;
  l_xd_    = l_x0_ ;

  // Calculate a Cartesian restoring force.
  r_xerr_.vel = r_x_.p - r_xd_.p;
  r_xerr_.rot = 0.5 * (r_xd_.M.UnitX() * r_x_.M.UnitX() +
                       r_xd_.M.UnitY() * r_x_.M.UnitY() +
                       r_xd_.M.UnitZ() * r_x_.M.UnitZ());

  l_xerr_.vel = l_x_.p - l_xd_.p;
  l_xerr_.rot = 0.5 * (l_xd_.M.UnitX() * l_x_.M.UnitX() +
                       l_xd_.M.UnitY() * l_x_.M.UnitY() +
                       l_xd_.M.UnitZ() * l_x_.M.UnitZ());


  // Force error
  r_ferr_(0) = r_ftData.wrench.force.x ;
  r_ferr_(1) = r_ftData.wrench.force.y ;
  r_ferr_(2) = r_ftData.wrench.force.z ;
  r_ferr_(3) = r_ftData.wrench.torque.x;
  r_ferr_(4) = r_ftData.wrench.torque.y;
  r_ferr_(5) = r_ftData.wrench.torque.z;

  l_ferr_(0) = l_ftData.wrench.force.x ;
  l_ferr_(1) = l_ftData.wrench.force.y ;
  l_ferr_(2) = l_ftData.wrench.force.z ;
  l_ferr_(3) = l_ftData.wrench.torque.x;
  l_ferr_(4) = l_ftData.wrench.torque.y;
  l_ferr_(5) = l_ftData.wrench.torque.z;


  l_buffer_indx = (l_buffer_indx+1)%10;
  r_buffer_indx = (r_buffer_indx+1)%10;

  l_force_buffer[l_buffer_indx] = l_ferr_(0);
  r_force_buffer[r_buffer_indx] = r_ferr_(0);

  l_ferr_(0) = 0;
  r_ferr_(0) = 0;
  for(int i = 0;i<10;i++)
  {
	  l_ferr_(0) += l_force_buffer[i]/10.0;
	  l_ferr_(0) += l_force_buffer[i]/10.0;
  }


  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    r_F_(i) = - Kp_(i) * r_xerr_(i) - Kd_(i) * r_xdot_(i);			// PD controller
    l_F_(i) = - Kp_(i) * l_xerr_(i) - Kd_(i) * l_xdot_(i);


// Note: ferr(1-3) are forces in x, y, z
//       ferr(4-6) are torques in x, y, z
//	  r_F_(i) = - Kp_(i) * r_xerr_(i) - Kd_(i) * r_xdot_(i) + restDGain*r_ferr_(i);
//	  l_F_(i) = - Kp_(i) * l_xerr_(i) - Kd_(i) * l_xdot_(i) + restDGain*l_ferr_(i);

  }


  if (forceTorque_on)
  {
	  for (unsigned int i = 0 ; i < 3 ; i++)	// ferr(0-2) are forces in x, y, z (ignore measured torques for now)
	  {
		  r_F_(i) +=  restDGain*r_ferr_(i);		// Add to compliance controller
		  l_F_(i) +=  restDGain*l_ferr_(i);		// ferr = f_actual - f_desired = f_actual so sign is positive
	  }

  }


  // Convert the force into a set of joint torques.
  for (unsigned int i = 0 ; i < r_kdl_chain_.getNrOfJoints() ; i++)
  {
    r_tau_(i) = 0;
    l_tau_(i) = 0;
    for (unsigned int j = 0 ; j < 6 ; j++)
    {
      r_tau_(i) += r_J_(j,i) * r_F_(j);
      l_tau_(i) += l_J_(j,i) * l_F_(j);
    }
  }

  // Force controller
  // something like this:
//  for (unsigned int j = 0 ; j < 6 ; j++)
//  {
//	r_tau_(j) += restDGain *r_ferr_(i); // or J*ferr?
//	l_tau_(j) += restDGain *l_ferr_(i);
//  }

  // Computed-torque control:
  // Restoring torque to keep arms at initial positions
  if(restPGain>=0.0)	// If negative, the arms will drift and activate the velocity controller
  {
	  for (unsigned int j = 0 ; j < 6 ; j++)
	  {
		r_tau_(j) += restPGain * (r_q0_(j) - r_q_(j));
		l_tau_(j) += restPGain * (l_q0_(j) - l_q_(j));
	  }
  }


  // Move arm and/or base:
	if( controller_on ) 						// TODO: include above code?
	{
		// And finally send these torques out.
		r_chain_.setEfforts(r_tau_);
		l_chain_.setEfforts(l_tau_);

		// Publish data in ROS message every 10 cycles (about 100Hz)
		if (++pub_cycle_count_ > 10)
		{
			should_publish_ = true;
			pub_cycle_count_ = 0;
		}

		if (should_publish_ && pubBaseMove_.trylock())
		{
			should_publish_ = false;

			// Compute base velocity: xdot_base = Kp*ep + Kv*ep_dot = Kp*ep + Kv*x_dot

			// Single arm stuff
/*			combinedPoseError = r_xerr_;
			combinedTwistError   = KDL::Twist::Zero();
*/
			// Double arm stuff

			combinedPoseError   = KDL::Twist::Zero();							// Combined position error: ep = x - xd
			combinedPoseError(0) = (r_xerr_.vel.x() + l_xerr_.vel.x())/2;
			combinedPoseError(1) = (r_xerr_.vel.y() + l_xerr_.vel.y())/2;
			combinedPoseError(5) = -atan((r_x_.p.x() - l_x_.p.x())/(r_x_.p.y() - l_x_.p.y()));

//			double r_xerr_theta = atan(r_xerr_.vel.x()/r_xerr_.vel.y());	// Doesn't work
//			double l_xerr_theta = atan(l_xerr_.vel.x()/l_xerr_.vel.y());
//			combinedPoseError(5) = (r_xerr_theta+l_xerr_theta)/2;

			// Old code:
//			combinedTwistError    = KDL::Twist::Zero();
//			combinedTwistError(0) = r_xdot_.vel.x();
//			combinedTwistError(1) = r_xdot_.vel.y();
//			combinedTwistError(5) = r_xdot_.rot.z();

			// Use average velocity of both arms
			combinedTwistError    = KDL::Twist::Zero();
			combinedTwistError(0) = (r_xdot_.vel.x() + l_xdot_.vel.x())/2;
			combinedTwistError(1) = (r_xdot_.vel.y() + l_xdot_.vel.y())/2;
			//combinedTwistError(5) = -atan((r_xdot_.vel.x() - l_xdot_.vel.x())/(r_xdot_.vel.y() - r_xdot_.vel.y()));
			//combinedTwistError(5) = (combinedPoseError(5) - prev_combinedPoseError(5)) / delT;


//			(r_x_.p.x() - l_x_.p.x())
//		(r_x_.p.y() - l_x_.p.y()
			 double u_prime;

			// d((a(t) - b(t))/(c(t)-d(t)),t)
//			if(abs((r_x_.p.y() - l_x_.p.y()))>0.001)
				u_prime =	((r_x_.p.y() - l_x_.p.y())*(r_xdot_.vel.x() - l_xdot_.vel.x()) - (r_x_.p.x() - l_x_.p.x())*(r_xdot_.vel.y() - r_xdot_.vel.y()))/( (r_x_.p.y() - l_x_.p.y())*(r_x_.p.y() - l_x_.p.y())   );
//			else
//			{
//				u_prime = 1000;
//			}

			combinedTwistError(5) = u_prime/(1 + combinedPoseError(5)*combinedPoseError(5));

			// This doesn't work:
//			double r_xdot_theta = atan(r_xdot_.vel.x()/r_xdot_.vel.y());
//			double l_xdot_theta = atan(l_xdot_.vel.x()/l_xdot_.vel.y());
//			combinedTwistError(5) = (r_xdot_theta+l_xdot_theta)/2;

			// Publish computed values
			if (realtime_pub_pose_error.trylock())
			{
				realtime_pub_pose_error.msg_.vel_x = combinedPoseError(0);
				realtime_pub_pose_error.msg_.vel_y = combinedPoseError(1);
				realtime_pub_pose_error.msg_.rot_z = combinedPoseError(5);
				realtime_pub_pose_error.unlockAndPublish();
			}
			if (realtime_pub_twist_error.trylock())
			{
				realtime_pub_twist_error.msg_.vel_x = combinedTwistError(0);
				realtime_pub_twist_error.msg_.vel_y = combinedTwistError(1);
				realtime_pub_twist_error.msg_.rot_z = combinedTwistError(5);
				realtime_pub_twist_error.unlockAndPublish();
			}


			pubBaseMove_.msg_.linear.x = pubBaseMove_.msg_.linear.y = pubBaseMove_.msg_.angular.z = 0;  // Set previous values to zero
			pubBaseMove_.msg_.linear.z = pubBaseMove_.msg_.angular.x = pubBaseMove_.msg_.angular.y = 0; // (these are not used but set to zero just in case)

			double combinedErrorRadius = sqrt(combinedPoseError.vel.x()*combinedPoseError.vel.x()
											+ combinedPoseError.vel.y()*combinedPoseError.vel.y()); 	// error in XY plane

			// Test if inside the circular threshold region.
			// Previously a xThresh and yThresh was used which caused wheel oscillations at the corner of the square threshold region
			if(combinedErrorRadius>rThresh)
			{
				if(combinedPoseError.vel.x()>0.005 || combinedPoseError.vel.x()<-0.005)
				{
					pubBaseMove_.msg_.linear.x = velPGain*combinedPoseError.vel.x()			// Uses same gains for x and y direction
												  + velDGain*combinedTwistError.vel.x();
				}

				if(combinedPoseError.vel.y()>0.005 || combinedPoseError.vel.y()<-0.005)
				{
				pubBaseMove_.msg_.linear.y = velPGain*combinedPoseError.vel.y()
										   + velDGain*combinedTwistError.vel.y();
				}
			}

			if( combinedPoseError.rot.z() > psiThresh ||  combinedPoseError.rot.z() < -psiThresh )
			{
				pubBaseMove_.msg_.angular.z = rotPGain*combinedPoseError.rot.z()
											+ rotDGain*combinedTwistError.rot.z();
			}

			pubBaseMove_.unlockAndPublish();
		}
	}

	// Capture controller data
	  int index = storage_index_;
	  if ((index >= 0) && (index < StoreLen))
	    {
//			storage_[index].restPGain_	 = restPGain;
//			storage_[index].restDGain_	 = restDGain;
//			storage_[index].velPGain_	 = velPGain;
//			storage_[index].velDGain_	 = velDGain;
//			storage_[index].rotPGain_	 = rotPGain;
//			storage_[index].rotDGain_	 = rotDGain;
//			storage_[index].rThresh_	 = rThresh;
//			storage_[index].psiThresh_	 = psiThresh;
//			storage_[index].Kp_vel_x_	 = Kp_.vel(0);
//			storage_[index].Kp_vel_y_	 = Kp_.vel(1);
//			storage_[index].Kp_vel_z_	 = Kp_.vel(2);
//			storage_[index].Kd_vel_x_	 = Kd_.vel(0);
//			storage_[index].Kd_vel_y_	 = Kd_.vel(1);
//			storage_[index].Kd_vel_z_	 = Kd_.vel(2);
//			storage_[index].Kp_rot_x_	 = Kp_.rot(0);
//			storage_[index].Kp_rot_y_	 = Kp_.rot(1);
//			storage_[index].Kp_rot_z_	 = Kp_.rot(2);
//			storage_[index].Kd_rot_x_	 = Kd_.rot(0);
//			storage_[index].Kd_rot_y_	 = Kd_.rot(1);
//			storage_[index].Kd_rot_z_	 = Kd_.rot(2);

		  storage_[index].vel_x	 = combinedPoseError.vel.x();
		  storage_[index].vel_y	 = combinedPoseError.vel.y();
		  storage_[index].rot_z	 = combinedPoseError.rot.z();

	      // Increment for the next cycle.
	      storage_index_ = index+1;
	    }

}

/***********************************************************************************************************************
Service call to capture and extract the data
***********************************************************************************************************************/
bool PR2CartClass::capture(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp)
{
  /* Record the starting time. */
  ros::Time started = ros::Time::now();

  /* Mark the buffer as clear (which will start storing). */
  storage_index_ = 0;

  /* Now wait until the buffer is full. */
  while (storage_index_ < StoreLen)
    {
      /* Sleep for 1ms as not to hog the CPU. */
      ros::Duration(0.001).sleep();

      /* Make sure we don't hang here forever. */
      if ( (ros::Time::now() - started) > ros::Duration(30.0))
        {
          ROS_ERROR("Waiting for buffer to fill up took longer than 30 seconds!");
          return false;
        }
    }

  /* Then we can publish the buffer contents. */
  int  index;
  for (index = 0 ; index < StoreLen ; index++)
    capture_pub_.publish(storage_[index]);

  return true;
}
/***********************************************************************************************************************
Service call to start controller
***********************************************************************************************************************/
bool PR2CartClass::start( std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp )
{
	ROS_INFO("Update loop is turned ON!");
	controller_on = true;
	return true;
}
/***********************************************************************************************************************
Service call to pause controller
***********************************************************************************************************************/
bool PR2CartClass::stop( std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp )
{
	ROS_INFO("Update loop is turned OFF!");
	controller_on = false;
	return true;
}
/***********************************************************************************************************************
Controller stopping in realtime
***********************************************************************************************************************/
void PR2CartClass::stopping()
{}


/***********************************************************************************************************************
Register controller to pluginlib
***********************************************************************************************************************/
PLUGINLIB_REGISTER_CLASS( PR2CartClass,
						  pr2_controller_ns::PR2CartClass,
                          pr2_controller_interface::Controller )
