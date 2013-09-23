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

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "ros/ros.h"
#include <urdf/model.h>

namespace pr2_controller_ns{

class PR2CartPushClass: public pr2_controller_interface::Controller
{
private:
  // The current robot state (to get the time stamp)
  pr2_mechanism_model::RobotState* robot_state_;

  // The chain of links and joints
  pr2_mechanism_model::Chain r_chain_;
  pr2_mechanism_model::Chain l_chain_;
  KDL::Chain r_kdl_chain_;
  KDL::Chain l_kdl_chain_;

  pr2_hardware_interface::AnalogIn* analogin_handle_;
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
  KDL::JntArray     r_q_;            // Joint positions
  KDL::JntArray     r_q0_;           // Joint initial positions
  KDL::JntArrayVel  r_qdot_;      // Joint velocities
  KDL::JntArray  	r_tau_;          // Joint torques
  KDL::JntArray  	tau_h;         // Joint torques from human

  KDL::JntArray     q_m_;          // Model Joint positions
  KDL::JntArray     qd_m_;         // Model Joint positions
  KDL::JntArray     qdd_m_;        // Model Joint positions

  KDL::JntArray     q_lower;       // Joint position lower limits
  KDL::JntArray     q_upper;       // Joint position upper limits
  KDL::JntArray     qd_limit;      // Joint velocity limits

  KDL::Frame        r_x_;            // Tip pose
  KDL::Frame        r_xd_;           // Tip desired pose
  KDL::Frame        r_x0_;           // Tip initial pose

  KDL::Twist        r_xerr_;         // Cart error
  KDL::Twist        r_xdot_;         // Cart velocity
  KDL::Wrench       r_F_;            // Cart effort
  KDL::Twist        ferr_;			// Cart effort error
  KDL::Jacobian     r_J_;            // Jacobian

  KDL::JntArray     l_q_;            // Joint positions
  KDL::JntArray     l_q0_;           // Joint initial positions
  KDL::JntArrayVel  l_qdot_;      // Joint velocities
  KDL::JntArray  	l_tau_;          // Joint torques
//  KDL::JntArray  	tau_h;         // Joint torques from human

//  KDL::JntArray     q_m_;          // Model Joint positions
//  KDL::JntArray     qd_m_;         // Model Joint positions
//  KDL::JntArray     qdd_m_;        // Model Joint positions
//
//  KDL::JntArray     q_lower;       // Joint position lower limits
//  KDL::JntArray     q_upper;       // Joint position upper limits
//  KDL::JntArray     qd_limit;      // Joint velocity limits

  KDL::Frame        l_x_;            // Tip pose
  KDL::Frame        l_xd_;           // Tip desired pose
  KDL::Frame        l_x0_;           // Tip initial pose

  KDL::Twist        l_xerr_;         // Cart error
  KDL::Twist        l_xdot_;         // Cart velocity
  KDL::Wrench       l_F_;            // Cart effort
//  KDL::Twist        ferr_;			// Cart effort error
  KDL::Jacobian     l_J_;            // Jacobian

  // Note the gains are incorrectly typed as a twist,
  // as there is no appropriate type!
  KDL::Twist     Kp_;           // Proportional gains
  KDL::Twist     Kd_;           // Derivative gains

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
