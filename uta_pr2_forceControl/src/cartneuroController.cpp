#include "uta_pr2_forceControl/cartneuroController.h"
#include <pluginlib/class_list_macros.h>
#include "oel/least_squares.hpp"

using namespace pr2_controller_ns;

/// Controller initialization in non-realtime
bool PR2CartneuroControllerClass::init(pr2_mechanism_model::RobotState *robot,
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

  double cartPos_Kp_x = 0;
  double cartPos_Kp_y = 0;
  double cartPos_Kp_z = 0;
  double cartPos_Kd_x = 0;
  double cartPos_Kd_y = 0;
  double cartPos_Kd_z = 0;

  double cartRot_Kp_x = 0;
  double cartRot_Kp_y = 0;
  double cartRot_Kp_z = 0;
  double cartRot_Kd_x = 0;
  double cartRot_Kd_y = 0;
  double cartRot_Kd_z = 0;

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
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

  // Resize (pre-allocate) the variables in non-realtime.
  q_.resize(kdl_chain_.getNrOfJoints());
  q0_.resize(kdl_chain_.getNrOfJoints());
  qdot_.resize(kdl_chain_.getNrOfJoints());
  tau_.resize(kdl_chain_.getNrOfJoints());
  J_.resize(kdl_chain_.getNrOfJoints());

  // Pick the gains.
//  Kp_.vel(0) = 100.0;  Kd_.vel(0) = 1.0;        // Translation x
//  Kp_.vel(1) = 000.0;  Kd_.vel(1) = 1.0;        // Translation y
//  Kp_.vel(2) = 100.0;  Kd_.vel(2) = 1.0;        // Translation z
//  Kp_.rot(0) = 100.0;  Kd_.rot(0) = 1.0;        // Rotation    x
//  Kp_.rot(1) = 100.0;  Kd_.rot(1) = 1.0;        // Rotation    y
//  Kp_.rot(2) = 100.0;  Kd_.rot(2) = 1.0;        // Rotation    z

  Kp_.vel(0) = cartPos_Kp_x;  Kd_.vel(0) = cartPos_Kd_x; // Translation x
  Kp_.vel(1) = cartPos_Kp_y;  Kd_.vel(1) = cartPos_Kd_y; // Translation y
  Kp_.vel(2) = cartPos_Kp_z;  Kd_.vel(2) = cartPos_Kd_z; // Translation z
  Kp_.rot(0) = cartRot_Kp_x;  Kd_.rot(0) = cartRot_Kd_x; // Rotation    x
  Kp_.rot(1) = cartRot_Kp_y;  Kd_.rot(1) = cartRot_Kd_y; // Rotation    y
  Kp_.rot(2) = cartRot_Kp_z;  Kd_.rot(2) = cartRot_Kd_z; // Rotation    z

  ROS_ERROR("Joint no: %d", kdl_chain_.getNrOfJoints());

  // TEST

  test_object_.updateDouble( 10 );

  Eigen::Matrix<double, 6, 1> & eigen_vector;
  eigen_vector.Zero();
  test_object_.updateEigen( eigen_vector );

  // TEST END

  return true;
}

/// Controller startup in realtime
void PR2CartneuroControllerClass::starting()
{

  test_object_.updateDouble( 11 );

  // Get the current joint values to compute the initial tip location.
  chain_.getPositions(q0_);
  jnt_to_pose_solver_->JntToCart(q0_, x0_);

  // Initialize the phase of the circle as zero.
  circle_phase_ = 0.0;

  // Also reset the time-of-last-servo-cycle.
  last_time_ = robot_state_->getTime();
}


/// Controller update loop in realtime
void PR2CartneuroControllerClass::update()
{

  test_object_.updateDouble( 12 );

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
  circle(2) = 0.1; //0.1 * sin(circle_phase_);
  circle(1) = 0.1; //0.1 * (cos(circle_phase_) - 1);

  xd_ = x0_;
  xd_.p += circle;
  xd_.M = KDL::Rotation::RPY( 0, 1.57079632679, 0 );


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
void PR2CartneuroControllerClass::stopping()
{}

// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS( PR2CartneuroControllerClass,
						  pr2_controller_ns::PR2CartneuroControllerClass,
                          pr2_controller_interface::Controller )
