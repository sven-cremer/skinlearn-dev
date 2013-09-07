#include "uta_pr2_forceControl/explforceController.h"
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;

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

	  // Force control only Z
	//      F_(2) = - 1 * ferr_(2); // - Kd_(i) * xdot_(i);

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


//PLUGINLIB_DECLARE_CLASS( uta_pr2_forceControl,PR2CartControllerClass,
//                         pr2_controller_ns::PR2CartControllerClass,
//                         pr2_controller_interface::Controller )


// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS( PR2ExplforceControllerClass,
                          pr2_controller_ns::PR2ExplforceControllerClass,
                          pr2_controller_interface::Controller )
