#include "uta_pr2_forceControl/explforceController.h"
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;

/// Controller update loop in realtime
void PR2ExplforceControllerClass::update()
{

  setFTData();
  pubFTData();

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

//  for (unsigned int i = 0 ; i < 6 ; i++)
//  {
//    xdot_(i) = 0;
//    for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++)
//      xdot_(i) += J_(i,j) * qdot_.qdot(j);
//  }
//
//  // Follow a circle of 10cm at 3 rad/sec.
//  circle_phase_ += 3.0 * dt;
//  KDL::Vector  circle(0,0,0);
//  circle(2) = 0.1 * sin(circle_phase_);
//  circle(1) = 0.1 * (cos(circle_phase_) - 1);
//
//  xd_ = x0_;
//  xd_.p += circle;
//
//  // Calculate a Cartesian restoring force.
//  xerr_.vel = x_.p - xd_.p;
//  xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
//                     xd_.M.UnitY() * x_.M.UnitY() +
//                     xd_.M.UnitZ() * x_.M.UnitZ());


  // Force error
    xerr_(0) = r_ftData.wrench.force.x ;
    xerr_(1) = r_ftData.wrench.force.y ;
    xerr_(2) = r_ftData.wrench.force.z ;
    xerr_(3) = r_ftData.wrench.torque.x;
    xerr_(4) = r_ftData.wrench.torque.y;
    xerr_(5) = r_ftData.wrench.torque.z;

  for (unsigned int i = 0 ; i < 6 ; i++)
	F_(i) = - Kp_(i) * xerr_(i); // - Kd_(i) * xdot_(i);
//    F_(i) = - Kp_(i) * xerr_(i) - Kd_(i) * xdot_(i);

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

// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS( PR2ExplforceControllerClass,
		          	      pr2_controller_ns::PR2ExplforceControllerClass,
                          pr2_controller_interface::Controller )
