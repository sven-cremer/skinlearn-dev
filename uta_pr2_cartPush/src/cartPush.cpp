#include "uta_pr2_cartPush/cartPush.h"
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;

/// Controller initialization in non-realtime
bool PR2CartPushClass::init( pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n )
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

  // Pick the gains.
  Kp_.vel(0) = 100.0;  Kd_.vel(0) = 1.0;        // Translation x
  Kp_.vel(1) = 100.0;  Kd_.vel(1) = 1.0;        // Translation y
  Kp_.vel(2) = 100.0;  Kd_.vel(2) = 1.0;        // Translation z
  Kp_.rot(0) = 100.0;  Kd_.rot(0) = 1.0;        // Rotation x
  Kp_.rot(1) = 100.0;  Kd_.rot(1) = 1.0;        // Rotation y
  Kp_.rot(2) = 100.0;  Kd_.rot(2) = 1.0;        // Rotation z



  /* get a handle to the hardware interface */
  pr2_hardware_interface::HardwareInterface* hardwareInterface = robot->model_->hw_;
  if(!hardwareInterface)
      ROS_ERROR("Something wrong with the hardware interface pointer!");

//  l_ft_handle_ = hardwareInterface->getForceTorque("l_gripper_motor");
//  r_ft_handle_ = hardwareInterface->getForceTorque("r_gripper_motor");
//
//  if( !l_ft_handle_ /*wristFTdata.getLeftHandle()*/ )
//      ROS_ERROR("Something wrong with getting l_ft handle");
//  if( !r_ft_handle_ /*wristFTdata.getRightHandle()*/ )
//      ROS_ERROR("Something wrong with getting r_ft handle");

  pub_cycle_count_ = 0;
  should_publish_  = false;

  // Initialize realtime publisher to publish to ROS topic
  pubBaseMove_.init(node,"base_controller/command", 1);

  return true;
}

/// Controller startup in realtime
void PR2CartPushClass::starting()
{
  // Get the current joint values to compute the initial tip location.
  r_chain_.getPositions(r_q0_);
  r_jnt_to_pose_solver_->JntToCart(r_q0_, r_x0_);

  l_chain_.getPositions(l_q0_);
  l_jnt_to_pose_solver_->JntToCart(l_q0_, l_x0_);

  // Also reset the time-of-last-servo-cycle.
  last_time_ = robot_state_->getTime();

//  // set FT sensor bias due to gravity
//  std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
//  l_ft_samples    = l_ftData_vector.size() - 1;
//  l_ftBias.wrench = l_ftData_vector[l_ft_samples];
//
//  std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
//  r_ft_samples    = r_ftData_vector.size() - 1;
//  r_ftBias.wrench = r_ftData_vector[r_ft_samples];

}


/// Controller update loop in realtime
void PR2CartPushClass::update()
{

//	std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
//	l_ft_samples    = l_ftData_vector.size() - 1;
////      l_ftData.wrench = l_ftData_vector[l_ft_samples];
//	l_ftData.wrench.force.x  = l_ftData_vector[l_ft_samples].force.x  - l_ftBias.wrench.force.x ;
//	l_ftData.wrench.force.y  = l_ftData_vector[l_ft_samples].force.y  - l_ftBias.wrench.force.y ;
//	l_ftData.wrench.force.z  = l_ftData_vector[l_ft_samples].force.z  - l_ftBias.wrench.force.z ;
//	l_ftData.wrench.torque.x = l_ftData_vector[l_ft_samples].torque.x - l_ftBias.wrench.torque.x;
//	l_ftData.wrench.torque.y = l_ftData_vector[l_ft_samples].torque.y - l_ftBias.wrench.torque.y;
//	l_ftData.wrench.torque.z = l_ftData_vector[l_ft_samples].torque.z - l_ftBias.wrench.torque.z;
//
//	std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
//	r_ft_samples    = r_ftData_vector.size() - 1;
////      r_ftData.wrench = r_ftData_vector[r_ft_samples];
//	r_ftData.wrench.force.x  = r_ftData_vector[r_ft_samples].force.x  - r_ftBias.wrench.force.x ;
//	r_ftData.wrench.force.y  = r_ftData_vector[r_ft_samples].force.y  - r_ftBias.wrench.force.y ;
//	r_ftData.wrench.force.z  = r_ftData_vector[r_ft_samples].force.z  - r_ftBias.wrench.force.z ;
//	r_ftData.wrench.torque.x = r_ftData_vector[r_ft_samples].torque.x - r_ftBias.wrench.torque.x;
//	r_ftData.wrench.torque.y = r_ftData_vector[r_ft_samples].torque.y - r_ftBias.wrench.torque.y;
//	r_ftData.wrench.torque.z = r_ftData_vector[r_ft_samples].torque.z - r_ftBias.wrench.torque.z;


  double dt;                    // Servo loop time step

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


  r_xd_    = r_x0_    ;
  l_xd_    = l_x0_    ;

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
  ferr_(0) = r_ftData.wrench.force.x ;
  ferr_(1) = r_ftData.wrench.force.y ;
  ferr_(2) = r_ftData.wrench.force.z ;
  ferr_(3) = r_ftData.wrench.torque.x;
  ferr_(4) = r_ftData.wrench.torque.y;
  ferr_(5) = r_ftData.wrench.torque.z;


  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    r_F_(i) = - Kp_(i) * r_xerr_(i) - Kd_(i) * r_xdot_(i);
    l_F_(i) = - Kp_(i) * l_xerr_(i) - Kd_(i) * l_xdot_(i);
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

//		pub_.msg_.header.stamp = robot_state_->getTime();
//		pub_.msg_.wrench = r_ftData.wrench; // wristFTdata.getRightData().wrench;
//
//		pubModelStates_.msg_ = modelState;
//
//		pub_.unlockAndPublish();
//		pubModelStates_.unlockAndPublish();

		double velGain = 3;

		if( r_xerr_.vel.x() > 0.02 ||  r_xerr_.vel.x() < -0.02)
		{
			pubBaseMove_.msg_.linear.x = velGain*r_xerr_.vel.x();
		}
		else
		{
			pubBaseMove_.msg_.linear.x = 0;
		}

		if( r_xerr_.vel.y() > 0.02 ||  r_xerr_.vel.y() < -0.02)
		{
			pubBaseMove_.msg_.linear.y = velGain*r_xerr_.vel.y();
		}
		else
		{
			pubBaseMove_.msg_.linear.y = 0;
		}

		pubBaseMove_.msg_.linear.z = 0;

		pubBaseMove_.msg_.angular.x = 0;
		pubBaseMove_.msg_.angular.y = 0;
		pubBaseMove_.msg_.angular.z = 0;

		pubBaseMove_.unlockAndPublish();

	}

}


/// Controller stopping in realtime
void PR2CartPushClass::stopping()
{}

// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS( PR2CartPushClass,
						  pr2_controller_ns::PR2CartPushClass,
                          pr2_controller_interface::Controller )
