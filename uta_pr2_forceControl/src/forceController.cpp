#include "uta_pr2_forceControl/forceController.h"
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;

/// Controller initialization in non-realtime
bool PR2ForceControllerClass::init( pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n )
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
  J_.resize(kdl_chain_.getNrOfJoints());

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

  return true;
}

/// Controller startup in realtime
void PR2ForceControllerClass::starting()
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

void PR2ForceControllerClass::setFTData()
{

//  wristFTdata.update();
	std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
	l_ft_samples    = l_ftData_vector.size() - 1;
//	l_ftData.wrench = l_ftData_vector[l_ft_samples];
	l_ftData.wrench.force.x  = l_ftData_vector[l_ft_samples].force.x  - l_ftBias.wrench.force.x ;
	l_ftData.wrench.force.y  = l_ftData_vector[l_ft_samples].force.y  - l_ftBias.wrench.force.y ;
	l_ftData.wrench.force.z  = l_ftData_vector[l_ft_samples].force.z  - l_ftBias.wrench.force.z ;
	l_ftData.wrench.torque.x = l_ftData_vector[l_ft_samples].torque.x - l_ftBias.wrench.torque.x;
	l_ftData.wrench.torque.y = l_ftData_vector[l_ft_samples].torque.y - l_ftBias.wrench.torque.y;
	l_ftData.wrench.torque.z = l_ftData_vector[l_ft_samples].torque.z - l_ftBias.wrench.torque.z;

	std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
	r_ft_samples    = r_ftData_vector.size() - 1;
//	r_ftData.wrench = r_ftData_vector[r_ft_samples];
	r_ftData.wrench.force.x  = r_ftData_vector[r_ft_samples].force.x  - r_ftBias.wrench.force.x ;
	r_ftData.wrench.force.y  = r_ftData_vector[r_ft_samples].force.y  - r_ftBias.wrench.force.y ;
	r_ftData.wrench.force.z  = r_ftData_vector[r_ft_samples].force.z  - r_ftBias.wrench.force.z ;
	r_ftData.wrench.torque.x = r_ftData_vector[r_ft_samples].torque.x - r_ftBias.wrench.torque.x;
	r_ftData.wrench.torque.y = r_ftData_vector[r_ft_samples].torque.y - r_ftBias.wrench.torque.y;
	r_ftData.wrench.torque.z = r_ftData_vector[r_ft_samples].torque.z - r_ftBias.wrench.torque.z;

}

void PR2ForceControllerClass::pubFTData()
{
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

      pub_.unlockAndPublish();
    }
}




/// Controller stopping in realtime
void PR2ForceControllerClass::stopping()
{}

