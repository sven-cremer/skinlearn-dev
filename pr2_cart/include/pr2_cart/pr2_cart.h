/***********************************************************************************************************************
FILENAME:    pr2_cart.h
AUTHORS:     Sven Cremer 		sven.cremer@mavs.uta.edu
             Isura Ranatunga	isura.ranatunga@mavs.uta.edu
			 University of Texas at Arlington, Copyright (C) 2013.

DESCRIPTION:
Realtime impedance and velocity controller for PR2 cart behavior

REFERENCES:
-This is a modified version of Isura's uta_pr2_cartpush package.
-"Writing a realtime Cartesian controller" ROS tutorial:
http://wiki.ros.org/pr2_mechanism/Tutorials/Writing%20a%20realtime%20Cartesian%20controller
-"Capturing data from a controller" ROS tutorial:
http://wiki.ros.org/pr2_mechanism/Tutorials/Capturing%20data%20from%20a%20controller

PUBLISHES:  NA
SUBSCRIBES: NA
SERVICES:   NA

REVISION HISTORY:
2014.02.07  SC     original file creation
2014.02.14  SC     code cleanup
2014.02.17  SC     capturing data from a controller
2015.10.26  SC     ported to hydro
***********************************************************************************************************************/

#include <pr2_controller_interface/controller.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include "realtime_tools/realtime_publisher.h"

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/JointState.h"

#include <std_srvs/Empty.h>
#include <pr2_mechanism_msgs/SwitchController.h>

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "ros/ros.h"
#include <urdf/model.h>

// Service messages for chaining gains
#include <ice_msgs/setValue.h>
#include <ice_msgs/setGains.h>
#include <ice_msgs/getState.h>

#include <ice_msgs/controllerState.h>
#include <ice_msgs/combinedTwistError.h>
#include <ice_msgs/combinedError.h>


namespace pr2_controller_ns{

enum
{
  StoreLen = 10000
};


class PR2CartClass: public pr2_controller_interface::Controller
{
private:
	// Proportional gains for base velocity controller
	double velPGain;
	double rotPGain;
	// Derivative gains for base velocity controller
	double velDGain;
	double rotDGain;
	// Thresholds for base velocity controller
	double rThresh;
	double psiThresh;

	// Gains for torque controller
	double restPGain;
	double restDGain;

	// Method that will get called when the service is called.
	bool set_velPGain	(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp);
	bool set_velDGain	(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp);
	bool set_rotPGain	(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp);
	bool set_rotDGain	(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp);
	bool set_rThresh	(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp);
	bool set_psiThresh	(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp);

	bool set_restPGain	(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp);
	bool set_restDGain	(ice_msgs::setValue::Request& req, ice_msgs::setValue::Response& resp);

	bool set_Kp_vel	(ice_msgs::setGains::Request& req, ice_msgs::setGains::Response& resp);
	bool set_Kd_vel	(ice_msgs::setGains::Request& req, ice_msgs::setGains::Response& resp);
	bool set_Kp_rot	(ice_msgs::setGains::Request& req, ice_msgs::setGains::Response& resp);
	bool set_Kd_rot	(ice_msgs::setGains::Request& req, ice_msgs::setGains::Response& resp);


	ros::ServiceServer srv_velPGain;
	ros::ServiceServer srv_velDGain;
	ros::ServiceServer srv_rotPGain;
	ros::ServiceServer srv_rotDGain;
	ros::ServiceServer srv_rThresh;
	ros::ServiceServer srv_psiThresh;

	ros::ServiceServer srv_restPGain;
	ros::ServiceServer srv_restDGain;

	ros::ServiceServer srv_Kp_vel;
	ros::ServiceServer srv_Kd_vel;
	ros::ServiceServer srv_Kp_rot;
	ros::ServiceServer srv_Kd_rot;


	ros::ServiceServer srv_getState;
	bool get_State(ice_msgs::getState::Request& req, ice_msgs::getState::Response& resp);

  // The current robot state (to get the time stamp)
  pr2_mechanism_model::RobotState* robot_state_;

  // The chain of links and joints
  pr2_mechanism_model::Chain r_chain_;
  pr2_mechanism_model::Chain l_chain_;
  KDL::Chain r_kdl_chain_;
  KDL::Chain l_kdl_chain_;

  pr2_hardware_interface::AnalogIn*      analogin_handle_     ;
  pr2_hardware_interface::Accelerometer* accelerometer_handle_;

  pr2_hardware_interface::ForceTorque* l_ft_handle_;
  pr2_hardware_interface::ForceTorque* r_ft_handle_;

  int l_ft_samples;
  int r_ft_samples;

  geometry_msgs::WrenchStamped l_ftBias;
  geometry_msgs::WrenchStamped r_ftBias;

  geometry_msgs::WrenchStamped l_ftData;
  geometry_msgs::WrenchStamped r_ftData;

  // KDL Solvers performing the actual computations
  boost::scoped_ptr<KDL::ChainFkSolverPos>    r_jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> r_jnt_to_jac_solver_;
  boost::scoped_ptr<KDL::ChainFkSolverPos>    l_jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> l_jnt_to_jac_solver_;

  // The variables (which need to be pre-allocated).
  KDL::JntArray     r_q_;       // Joint positions
  KDL::JntArray     r_q0_;      // Joint initial positions
  KDL::JntArrayVel  r_qdot_;    // Joint velocities
  KDL::JntArray     r_tau_;     // Joint torques
  KDL::JntArray     tau_h;      // Joint torques from human

  KDL::JntArray     q_m_;       // Model Joint positions
  KDL::JntArray     qd_m_;      // Model Joint positions
  KDL::JntArray     qdd_m_;     // Model Joint positions

  KDL::JntArray     q_lower;    // Joint position lower limits
  KDL::JntArray     q_upper;    // Joint position upper limits
  KDL::JntArray     qd_limit;   // Joint velocity limits

  KDL::Frame        r_x_;       // Tip pose
  KDL::Frame        r_xd_;      // Tip desired pose
  KDL::Frame        r_x0_;      // Tip initial pose

  KDL::Twist        r_xerr_;    // Cart error			TODO: use KDL::Frame instead
  KDL::Twist        r_xdot_;    // Cart velocity
  KDL::Wrench       r_F_;       // Cart effort
  KDL::Twist        r_ferr_;	// Cart effort error
  KDL::Twist        l_ferr_;	// Cart effort error
  KDL::Jacobian     r_J_;       // Jacobian

  KDL::JntArray     l_q_;       // Joint positions
  KDL::JntArray     l_q0_;      // Joint initial positions
  KDL::JntArrayVel  l_qdot_;    // Joint velocities
  KDL::JntArray     l_tau_;     // Joint torques
//  KDL::JntArray   tau_h;      // Joint torques from human

//  KDL::JntArray   q_m_;       // Model Joint positions
//  KDL::JntArray   qd_m_;      // Model Joint positions
//  KDL::JntArray   qdd_m_;     // Model Joint positions
//
//  KDL::JntArray   q_lower;    // Joint position lower limits
//  KDL::JntArray   q_upper;    // Joint position upper limits
//  KDL::JntArray   qd_limit;   // Joint velocity limits

  KDL::Frame        l_x_;       // Tip pose
  KDL::Frame        l_xd_;      // Tip desired pose
  KDL::Frame        l_x0_;      // Tip initial pose

  KDL::Twist        l_xerr_;    // Cart error
  KDL::Twist        l_xdot_;    // Cart velocity
  KDL::Wrench       l_F_;       // Cart effort
//  KDL::Twist      ferr_;		// Cart effort error
  KDL::Jacobian     l_J_;       // Jacobian

  KDL::Twist		combinedPoseError  ; // pose error of both arms
  KDL::Twist		combinedTwistError ; // twist error of both arms

  // Note the gains are incorrectly typed as a twist,
  // as there is no appropriate type!
  KDL::Twist        Kp_;        // Proportional gains
  KDL::Twist        Kd_;        // Derivative gains

  // The trajectory variables
  double    circle_phase_;      // Phase along the circle
  ros::Time last_time_;         // Time of the last servo cycle

  //! realtime publisher for max_force value
  realtime_tools::RealtimePublisher<geometry_msgs::Twist> pubBaseMove_;

  geometry_msgs::WrenchStamped r_forceData;

  //! publish max_force values every X realtime cycles
  int pub_cycle_count_;
  bool should_publish_;

  urdf::Model urdf_model;

  ros::NodeHandle node;

  bool controller_on;

  bool forceTorque_on;
  
  double l_force_buffer[10];
  double r_force_buffer[10];
  int l_buffer_indx;
  int r_buffer_indx;

  double dt;                    // Servo loop time step

  bool start( std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp );
  bool stop(  std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp );



	// Capturing data from controller
	  bool capture(std_srvs::Empty::Request& req, std_srvs::Empty::Response& resp);
	  ros::ServiceServer capture_srv_;
	  ros::Publisher     capture_pub_;
	  ice_msgs::combinedTwistError  storage_[StoreLen];
	  volatile int storage_index_;

	  //Realtime publisher
	  realtime_tools::RealtimePublisher<ice_msgs::combinedError> realtime_pub_pose_error;
	  realtime_tools::RealtimePublisher<ice_msgs::combinedError> realtime_pub_twist_error;


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:
  bool init(pr2_mechanism_model::RobotState *robot,
            ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();

};
}

