#include "uta_pr2_forceControl/cartesianController.h"
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;


/// Controller initialization in non-realtime
bool PR2CartControllerClass::init(pr2_mechanism_model::RobotState *robot,
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
    ROS_ERROR("PR2CartController could not use the chain from '%s' to '%s'",
              root_name.c_str(), tip_name.c_str());
    return false;
  }

  // Store the robot handle for later use (to get time).
  robot_state_ = robot;

  // Construct the kdl solvers in non-realtime.
  KDL::Chain kdl_chain;
  chain_.toKDL(kdl_chain);
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain));
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain));

  // Resize (pre-allocate) the temporary variables in non-realtime.
  qtmp_.resize(chain_.getNumOfJoints());
  Jtmp_.resize(chain_.getNumOfJoints());

  // Verify the number of joints.
  if (chain_.getNumOfJoints() != Joints)
  {
    ROS_ERROR("The chain from '%s' to '%s' does not contain %d joints",
              root_name.c_str(), tip_name.c_str(), Joints);
    return false;
  }

  // Pick the gains.
  Kp_(0) = 100.0;  Kd_(0) = 1.0;        // Translation x
  Kp_(1) = 100.0;  Kd_(1) = 1.0;        // Translation y
  Kp_(2) = 100.0;  Kd_(2) = 1.0;        // Translation z
  Kp_(3) = 100.0;  Kd_(3) = 1.0;        // Rotation x
  Kp_(4) = 100.0;  Kd_(4) = 1.0;        // Rotation y
  Kp_(5) = 100.0;  Kd_(5) = 1.0;        // Rotation z

  return true;
}


/// Controller startup in realtime
void MyCartControllerClass::starting()
{
  // Get the current joint values to compute the initial tip location.
  chain_.getJointPositions(qtmp_);
  jnt_to_pose_solver_->JntToCart(qtmp_, xtmp_);
  kin_.KDLtoEigen(xtmp_, x0_);

  // Initialize the phase of the circle as zero.
  circle_phase_ = 0.0;

  // Also reset the time-of-last-servo-cycle.
  last_time_ = robot_state_->getTime();
}


/// Controller update loop in realtime
void MyCartControllerClass::update()
{
  double       dt;              // Servo loop time step
  JointVector  q, qdot, tau;    // Joint position, velocity, torques
  CartPose     x, xd;           // Tip pose, desired pose
  Cart6Vector  xerr, xdot, F;   // Cart error,velocity,effort
  JacobianMatrix  J;            // Jacobian

  // Calculate the dt between servo cycles.
  dt = (robot_state_->getTime() - last_time_).toSec();
  last_time_ = robot_state_->getTime();

  // Get the current joint positions and velocities.
  chain_.getJointPositions(q);
  chain_.getJointVelocities(qdot);

  // Compute the forward kinematics and Jacobian (at this location).
  kin_.EigentoKDL(q, qtmp_);
  jnt_to_pose_solver_->JntToCart(qtmp_, xtmp_);
  jnt_to_jac_solver_->JntToJac(qtmp_, Jtmp_);
  kin_.KDLtoEigen(xtmp_, x);
  kin_.KDLtoEigen(Jtmp_, J);

  xdot = J * qdot;

  // Follow a circle of 10cm at 3 rad/sec.
  circle_phase_ += 3.0 * dt;
  Cart3Vector  circle(0,0,0);
  circle.x() = 0.1 * sin(circle_phase_);
  circle.y() = 0.1 * (1 - cos(circle_phase_));

  xd = x0_;
  xd.translation() += circle;

  // Calculate a Cartesian restoring force.
  kin_.computeCartError(x,xd,xerr);
  F = - Kp_.asDiagonal() * xerr - Kd_.asDiagonal() * xdot;

  // Convert the force into a set of joint torques.
  tau = J.transpose() * F;

  // And finally send these torques out.
  chain_.setJointEfforts(tau);
}


/// Controller stopping in realtime
void MyCartControllerClass::stopping()
{}



/// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS(PR2CartControllerPlugin,
                         pr2_controller_ns::PR2CartControllerClass,
                         pr2_controller_interface::Controller)
