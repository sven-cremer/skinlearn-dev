#include "uta_pr2_forceControl/explforceController.h"
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;

/// Controller initialization in non-realtime
bool PR2ExplforceControllerClass::init( pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n )
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
  }
  ROS_INFO("Successfully parsed URDF file");

  // Store the robot handle for later use (to get time).
  robot_state_ = robot;

  // Construct the kdl solvers in non-realtime.
  chain_.toKDL(kdl_chain_);
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

  // Resize (pre-allocate) the variables in non-realtime.
  q_.resize(kdl_chain_.getNrOfJoints());
  q0_.resize(kdl_chain_.getNrOfJoints());
  qdot_.resize(kdl_chain_.getNrOfJoints());
  tau_.resize(kdl_chain_.getNrOfJoints());
  tau_h.resize(kdl_chain_.getNrOfJoints());

  q_m_.resize(kdl_chain_.getNrOfJoints());
  qd_m_.resize(kdl_chain_.getNrOfJoints());
  qdd_m_.resize(kdl_chain_.getNrOfJoints());

  q_lower.resize(kdl_chain_.getNrOfJoints());
  q_upper.resize(kdl_chain_.getNrOfJoints());
  qd_limit.resize(kdl_chain_.getNrOfJoints());

  J_.resize(kdl_chain_.getNrOfJoints());

  modelState.name.resize(kdl_chain_.getNrOfJoints());
  modelState.position.resize(kdl_chain_.getNrOfJoints());
  modelState.velocity.resize(kdl_chain_.getNrOfJoints());

  modelState.name[0] = kdl_chain_.getSegment(0).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[1] = kdl_chain_.getSegment(1).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[2] = kdl_chain_.getSegment(2).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[3] = kdl_chain_.getSegment(4).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[4] = kdl_chain_.getSegment(6).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[5] = kdl_chain_.getSegment(7).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[6] = kdl_chain_.getSegment(8).getJoint().getName(); // TODO test this stuff, better way to get joint names...

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

//	/////////////////////////
//	// System Model
//
//	double m = 5;
//	double d = 5;
//	double k = 5;
//
//	Mm << m, 0, 0, 0, 0, 0, 0,
//		  0, m, 0, 0, 0, 0, 0,
//		  0, 0, m, 0, 0, 0, 0,
//		  0, 0, 0, m, 0, 0, 0,
//		  0, 0, 0, 0, m, 0, 0,
//		  0, 0, 0, 0, 0, m, 0,
//		  0, 0, 0, 0, 0, 0, m;
//
//	Dm << d, 0, 0, 0, 0, 0, 0,
//		  0, d, 0, 0, 0, 0, 0,
//		  0, 0, d, 0, 0, 0, 0,
//		  0, 0, 0, d, 0, 0, 0,
//		  0, 0, 0, 0, d, 0, 0,
//		  0, 0, 0, 0, 0, d, 0,
//		  0, 0, 0, 0, 0, 0, d;
//
//	Km << k, 0, 0, 0, 0, 0, 0,
//		  0, k, 0, 0, 0, 0, 0,
//		  0, 0, k, 0, 0, 0, 0,
//		  0, 0, 0, k, 0, 0, 0,
//		  0, 0, 0, 0, k, 0, 0,
//		  0, 0, 0, 0, 0, k, 0,
//		  0, 0, 0, 0, 0, 0, k;
//
//	q_m   << 0, 0, 0, 0, 0, 0, 0 ;
//	qd_m  << 0, 0, 0, 0, 0, 0, 0 ;
//	qdd_m << 0, 0, 0, 0, 0, 0, 0 ;
//
//	t_h   << 0, 0, 0, 0, 0, 0, 0 ;
//
//	MmInv = Mm;
//
//	delT  = 0.001;
//
//	// System Model END
//	/////////////////////////
//
//
//	/////////////////////////
//	// NN
//
//	kappa  = 0.7;
//	Kv     = 10; // prop. gain for PID inner loop
//	lambda = 1; //*std::sqrt(Kp); // der. gain for PID inner loop
//	Kz     = 3;
//	Zb     = 100;
//
//	hiddenLayerIdentity.setIdentity();
//
//	W_trans.setZero();
//	W_trans_next.setZero();
//	V_trans.setZero();
//	V_trans_next.setZero();
//
//	F.setIdentity();
//	G.setIdentity();
//	L.setIdentity();
//
//	F = 20*F;
//	G = 10*G;
//
//	// NN END
//	/////////////////////////

  /* get a handle to the hardware interface */
  pr2_hardware_interface::HardwareInterface* hardwareInterface = robot->model_->hw_;
  if(!hardwareInterface)
      ROS_ERROR("Something wrong with the hardware interface pointer!");

  l_ft_handle_ = hardwareInterface->getForceTorque("l_gripper_motor");
  r_ft_handle_ = hardwareInterface->getForceTorque("r_gripper_motor");

  if( !l_ft_handle_ /*wristFTdata.getLeftHandle()*/ )
      ROS_ERROR("Something wrong with getting l_ft handle");
  if( !r_ft_handle_ /*wristFTdata.getRightHandle()*/ )
      ROS_ERROR("Something wrong with getting r_ft handle");

  pub_cycle_count_ = 0;
  should_publish_  = false;

  // Initialize realtime publisher to publish to ROS topic
  pub_.init(n, "force_torque_stats", 2);
  pubModelStates_.init(n, "model_joint_states", 2);

  return true;
}

/// Controller startup in realtime
void PR2ExplforceControllerClass::starting()
{
  // Get the current joint values to compute the initial tip location.
  chain_.getPositions(q0_);
  jnt_to_pose_solver_->JntToCart(q0_, x0_);

  // Initialize the phase of the circle as zero.
  circle_phase_ = 0.0;

  // Also reset the time-of-last-servo-cycle.
  last_time_ = robot_state_->getTime();

  // set FT sensor bias due to gravity
//  wristFTdata.setBias();
  std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
  l_ft_samples    = l_ftData_vector.size() - 1;
  l_ftBias.wrench = l_ftData_vector[l_ft_samples];

  std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
  r_ft_samples    = r_ftData_vector.size() - 1;
  r_ftBias.wrench = r_ftData_vector[r_ft_samples];

}


/// Controller update loop in realtime
void PR2ExplforceControllerClass::update()
{

//  wristFTdata.update();
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

  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    xdot_(i) = 0;
    for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++)
      xdot_(i) += J_(i,j) * qdot_.qdot(j);
  }

  // Follow a circle of 10cm at 3 rad/sec.
  circle_phase_ += 3.0 * dt;
  KDL::Vector  circle(0,0,0);
//  circle(2) = 0.1 * sin(circle_phase_);
//  circle(1) = 0.1 * (cos(circle_phase_) - 1);

  xd_ = x0_;
  xd_.p += circle;

  // Calculate a Cartesian restoring force.
  xerr_.vel = x_.p - xd_.p;
  xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
                     xd_.M.UnitY() * x_.M.UnitY() +
                     xd_.M.UnitZ() * x_.M.UnitZ());


  // Force error
  ferr_(0) = r_ftData.wrench.force.x ;
  ferr_(1) = r_ftData.wrench.force.y ;
  ferr_(2) = r_ftData.wrench.force.z ;
  ferr_(3) = r_ftData.wrench.torque.x;
  ferr_(4) = r_ftData.wrench.torque.y;
  ferr_(5) = r_ftData.wrench.torque.z;


  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    F_(i) = - Kp_(i) * xerr_(i) - Kd_(i) * xdot_(i);
  }

  // Force control only ferr Z in ft sensor frame is x in robot frame
  F_(0) = ferr_(2); // - Kd_(i) * xdot_(i);


  // Convert the force into a set of joint torques.
  for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++)
  {
    tau_(i) = 0;
    for (unsigned int j = 0 ; j < 6 ; j++)
    {
      tau_(i) += J_(j,i) * F_(j);
      tau_h(i)+= J_(j,i) * ferr_(j); // this will give the torque from interaction
    }
  }

    circle(2) = 0.1 * sin(circle_phase_);
    circle(1) = 0.1 * (cos(circle_phase_) - 1);

//    /////////////////////////
//	// System Model
//
//  	// Integrator
//	t_h(0) = tau_h(0);
//	t_h(1) = tau_h(1);
//	t_h(2) = tau_h(2);
//	t_h(3) = tau_h(3);
//	t_h(4) = tau_h(4);
//	t_h(5) = tau_h(5);
//	t_h(6) = tau_h(6);
//
//	// Current joint positions and velocities
//	q = JointKdl2Eigen( q_ );
//	qd = JointVelKdl2Eigen( qdot_ );
//
//	q_m   = q_m + delT*qd_m;
//	qd_m  = qd_m + delT*qdd_m;
//	qdd_m = MmInv*( t_h - Dm*qd_m - Km*q_m );
//
//	// Check for joint limits and reset
//	// (condition) ? (if_true) : (if_false)
//	q_m(0) = fmax( (double) q_m(0), (double) q_lower(0) );
//	q_m(1) = fmax( (double) q_m(1), (double) q_lower(1) );
//	q_m(2) = fmax( (double) q_m(2), (double) q_lower(2) );
//	q_m(3) = fmax( (double) q_m(3), (double) q_lower(3) );
////	q_m(4) = fmax( (double) q_m(4), (double) q_lower(4) );
//	q_m(5) = fmax( (double) q_m(5), (double) q_lower(5) );
////	q_m(6) = fmax( (double) q_m(6), (double) q_lower(6) );
//
//	q_m(0) = fmin( (double) q_m(0), (double) q_upper(0) );
//	q_m(1) = fmin( (double) q_m(1), (double) q_upper(1) );
//	q_m(2) = fmin( (double) q_m(2), (double) q_upper(2) );
//	q_m(3) = fmin( (double) q_m(3), (double) q_upper(3) );
////	q_m(4) = fmin( (double) q_m(4), (double) q_upper(4) );
//	q_m(5) = fmin( (double) q_m(5), (double) q_upper(5) );
////	q_m(6) = fmin( (double) q_m(6), (double) q_upper(6) );
//
//	qd_m(0) = fmin( (double) qd_m(0), (double) qd_limit(0) );
//	qd_m(1) = fmin( (double) qd_m(1), (double) qd_limit(1) );
//	qd_m(2) = fmin( (double) qd_m(2), (double) qd_limit(2) );
//	qd_m(3) = fmin( (double) qd_m(3), (double) qd_limit(3) );
//	qd_m(4) = fmin( (double) qd_m(4), (double) qd_limit(4) );
//	qd_m(5) = fmin( (double) qd_m(5), (double) qd_limit(5) );
//	qd_m(6) = fmin( (double) qd_m(6), (double) qd_limit(6) );
//
//	// System Model END
//	/////////////////////////
//
//
//    /////////////////////////
//	// NN
//
//	W_trans = W_trans_next;
//	V_trans = V_trans_next;
//
//	// Filtered error
//	r = lambda*(qd_m - qd) + (q_m - q);
//
//	// Robust term
//	Z.block(0,0,Hidden,Outputs) = W_trans.transpose();
//	Z.block(Hidden,Outputs,Inputs+1,Hidden) = V_trans.transpose();
//	vRobust = /*kappa*r.norm() */- Kz*(Z.norm() + Zb)*r;
//
//	x(0 ) =  q(0);
//	x(1 ) =  q(1);
//	x(2 ) =  q(2);
//	x(3 ) =  q(3);
//	x(4 ) =  q(4);
//	x(5 ) =  q(5);
//	x(6 ) =  q(6);
//	x(7 ) = qd(0);
//	x(8 ) = qd(1);
//	x(9 ) = qd(2);
//	x(10) = qd(3);
//	x(11) = qd(4);
//	x(12) = qd(5);
//	x(13) = qd(6);
//	x(14) = 1;
//
//	hiddenLayer_in = V_trans*x;
//	hiddenLayer_out = sigmoid(hiddenLayer_in);
//	outputLayer_out = W_trans*hiddenLayer_out;
//
//	y = outputLayer_out;
//
//	// control torques
//	tau = Kv*r + y /*- vRobust*/ - t_h;
//
//	//
//	sigmaPrime = hiddenLayer_out.asDiagonal()*( hiddenLayerIdentity - hiddenLayerIdentity*hiddenLayer_out.asDiagonal() );
//
//	// Wk+1                  = Wk                  +  Wkdot                                                                                                          * dt
//	W_trans_next.transpose() = W_trans.transpose() + (F*hiddenLayer_out*r.transpose() - F*sigmaPrime*V_trans*x*r.transpose() - kappa*F*r.norm()*W_trans.transpose()) * delT;
//
//	// Vk+1                  = Vk                  +  Vkdot                                                                                      			 * dt
//	V_trans_next.transpose() = V_trans.transpose() + (G*x*(sigmaPrime.transpose()*W_trans.transpose()*r).transpose() - kappa*G*r.norm()*V_trans.transpose()) * delT;
//
//	// Convert from Eigen to KDL
//	tau_ = JointEigen2Kdl( tau );
//
//	// NN END
//	/////////////////////////


	modelState.header.stamp = robot_state_->getTime();

	modelState.position[0] = 0; // q_m(0);
	modelState.position[1] = 0; // q_m(1);
	modelState.position[2] = 0; // q_m(2);
	modelState.position[3] = 0; // q_m(3);
	modelState.position[4] = 0; // q_m(4);
	modelState.position[5] = 0; // q_m(5);
	modelState.position[6] = 0; // q_m(6);

	modelState.velocity[0] = 0; // tau(0);
	modelState.velocity[1] = 0; // tau(1);
	modelState.velocity[2] = 0; // tau(2);
	modelState.velocity[3] = 0; // tau(3);
	modelState.velocity[4] = 0; // tau(4);
	modelState.velocity[5] = 0; // tau(5);
	modelState.velocity[6] = 0; // tau(6);

	// And finally send these torques out.
    chain_.setEfforts(tau_);

    // Publish data in ROS message every 10 cycles (about 100Hz)
	if (++pub_cycle_count_ > 10)
	{
		should_publish_ = true;
		pub_cycle_count_ = 0;
	}

	if (should_publish_ && pub_.trylock())
	{
		should_publish_ = false;

		pub_.msg_.header.stamp = robot_state_->getTime();
		pub_.msg_.wrench = r_ftData.wrench; // wristFTdata.getRightData().wrench;

		pubModelStates_.msg_ = modelState;

		pub_.unlockAndPublish();
		pubModelStates_.unlockAndPublish();
	}

}

//PR2ExplforceControllerClass::SystemVector
//PR2ExplforceControllerClass::JointKdl2Eigen( KDL::JntArray & joint_ )
//{
//	SystemVector joint;
//	joint(0) = joint_(0);
//	joint(1) = joint_(1);
//	joint(2) = joint_(2);
//	joint(3) = joint_(3);
//	joint(4) = joint_(4);
//	joint(5) = joint_(5);
//	joint(6) = joint_(6);
//
//	return joint;
//}
//
//PR2ExplforceControllerClass::SystemVector
//PR2ExplforceControllerClass::JointVelKdl2Eigen( KDL::JntArrayVel & joint_ )
//{
//	SystemVector joint;
//	joint(0) = joint_.qdot(0);
//	joint(1) = joint_.qdot(1);
//	joint(2) = joint_.qdot(2);
//	joint(3) = joint_.qdot(3);
//	joint(4) = joint_.qdot(4);
//	joint(5) = joint_.qdot(5);
//	joint(6) = joint_.qdot(6);
//
//	return joint;
//}
//
//KDL::JntArray
//PR2ExplforceControllerClass::JointEigen2Kdl( SystemVector & joint )
//{
//	KDL::JntArray joint_;
//	joint_.resize(7);
//
//	joint (0) = joint(0);
//	joint_(1) = joint(1);
//	joint_(2) = joint(2);
//	joint_(3) = joint(3);
//	joint_(4) = joint(4);
//	joint_(5) = joint(5);
//	joint_(6) = joint(6);
//
//	return joint_;
//}
//
//Eigen::Matrix<double, PR2ExplforceControllerClass::Hidden, 1>
//PR2ExplforceControllerClass::sigmoid( Eigen::Matrix<double, Hidden, 1> & z )
//{
//  for(uint i=0;i<z.size();i++)
//  {
//	z(i) = 1.0/(1.0 + exp(-(double)z(i)));
//  }
//  return z;
//}



/// Controller stopping in realtime
void PR2ExplforceControllerClass::stopping()
{}


//PLUGINLIB_DECLARE_CLASS( uta_pr2_forceControl,PR2CartControllerClass,
//                         pr2_controller_ns::PR2CartControllerClass,
//                         pr2_controller_interface::Controller )


// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS( PR2ExplforceControllerClass,
                          pr2_controller_ns::PR2ExplforceControllerClass,
                          pr2_controller_interface::Controller )
