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

  q_m_.resize(kdl_chain_.getNrOfJoints());
  qd_m_.resize(kdl_chain_.getNrOfJoints());
  qdd_m_.resize(kdl_chain_.getNrOfJoints());

  J_.resize(kdl_chain_.getNrOfJoints());

//  modelState.name.resize(kdl_chain_.getNrOfJoints());
//  modelState.position.resize(kdl_chain_.getNrOfJoints());
//  modelState.velocity.resize(kdl_chain_.getNrOfJoints());

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



  l_ft_handle_ = hardwareInterface->getForceTorque("l_gripper_motor");
  r_ft_handle_ = hardwareInterface->getForceTorque("r_gripper_motor");

//  wristFTdata.setLeftHandle( l_ft_handle_ );
//  wristFTdata.setRightHandle( r_ft_handle_ );

  if( !l_ft_handle_ /*wristFTdata.getLeftHandle()*/ )
      ROS_ERROR("Something wrong with getting l_ft handle");
  if( !r_ft_handle_ /*wristFTdata.getRightHandle()*/ )
      ROS_ERROR("Something wrong with getting r_ft handle");

  pub_cycle_count_ = 0;
  should_publish_  = false;

  // Initialize realtime publisher to publish to ROS topic
  pub_.init(n, "force_torque_stats", 2);
//  pubModelStates_.init(n, "model_joint_states", 2);

//  *testClass = SystemModel( 10, 10, 10);

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
      tau_(i) += J_(j,i) * F_(j);
  }


//	testClass->update( tau_ );
//	testClass->getStates( q_m_, qd_m_, qdd_m_ );

//	modelState.header.stamp = robot_state_->getTime();
//
//	modelState.position.push_back(q_m_(0));
//	modelState.position.push_back(q_m_(1));
//	modelState.position.push_back(q_m_(2));
//	modelState.position.push_back(q_m_(3));
//	modelState.position.push_back(q_m_(4));
//	modelState.position.push_back(q_m_(5));
//	modelState.position.push_back(q_m_(6));
//
//	modelState.velocity.push_back(qd_m_(0));
//	modelState.velocity.push_back(qd_m_(1));
//	modelState.velocity.push_back(qd_m_(2));
//	modelState.velocity.push_back(qd_m_(3));
//	modelState.velocity.push_back(qd_m_(4));
//	modelState.velocity.push_back(qd_m_(5));
//	modelState.velocity.push_back(qd_m_(6));


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

//		pubModelStates_.msg_ = modelState;

		pub_.unlockAndPublish();
//		pubModelStates_.unlockAndPublish();
	}

}


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
