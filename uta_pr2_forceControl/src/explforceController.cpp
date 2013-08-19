#include "uta_pr2_forceControl/explforceController.h"
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;


/// Controller initialization in non-realtime
bool PR2ExplforceControllerClass::init(pr2_mechanism_model::RobotState *robot,
                                 ros::NodeHandle &n)
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

  /* get a handle to the left gripper accelerometer */
  accelerometer_handle_ = hardwareInterface->getAccelerometer("l_gripper_motor");
  if(!accelerometer_handle_)
      ROS_ERROR("Something wrong with getting accelerometer handle");

  // set to 1.5 kHz bandwidth (should be the default)
  accelerometer_handle_->command_.bandwidth_ = 6;

  // set to +/- 8g range (0=2g,1=4g)
  accelerometer_handle_->command_.range_ = 2;





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
}


/// Controller update loop in realtime
void PR2ExplforceControllerClass::update()
{




// retrieve our accelerometer data
  // it is most likely that this loop below will run 3 times,
  // this is because the ROS controller_manager calls this
  // function at 1khz and data is buffered from the accelerometer
  // at 3khz.
  std::vector<geometry_msgs::Vector3> threeAccs = accelerometer_handle_->state_.samples_;
  for( uint  i = 0; i < threeAccs.size(); i++ )
  {
	// here is where you would do anything that you want with
	// the measured acceleration data. This is a good place to
	// do any filtering or other such manipulation. Below we simply
	// overwrite global variables that act as storage containers.
	aX = threeAccs[i].x;
	aY = threeAccs[i].y;
	aZ = threeAccs[i].z;
  }

  ROS_WARN_STREAM("XYZ: "<< aX << " "<< aY << " "<< aZ);


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
  circle(2) = 0.1 * sin(circle_phase_);
  circle(1) = 0.1 * (cos(circle_phase_) - 1);

  xd_ = x0_;
  xd_.p += circle;

  // Calculate a Cartesian restoring force.
  xerr_.vel = x_.p - xd_.p;
  xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
                     xd_.M.UnitY() * x_.M.UnitY() +
                     xd_.M.UnitZ() * x_.M.UnitZ());

  for (unsigned int i = 0 ; i < 6 ; i++)
    F_(i) = - Kp_(i) * xerr_(i) - Kd_(i) * xdot_(i);

  // Convert the force into a set of joint torques.
  for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++)
  {
    tau_(i) = 0;
    for (unsigned int j = 0 ; j < 6 ; j++)
      tau_(i) += J_(j,i) * F_(j);
  }

  // And finally send these torques out.
  chain_.setEfforts(tau_);
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