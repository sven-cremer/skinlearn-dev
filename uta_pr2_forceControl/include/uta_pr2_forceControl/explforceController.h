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

#include "sensor_msgs/JointState.h"

#include <Eigen/Geometry>

namespace pr2_controller_ns{

class PR2ExplforceControllerClass: public pr2_controller_interface::Controller
{
private:
  // The current robot state (to get the time stamp)
  pr2_mechanism_model::RobotState* robot_state_;

  // The chain of links and joints
  pr2_mechanism_model::Chain chain_;
  KDL::Chain kdl_chain_;

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

  sensor_msgs::JointState modelState;

  // KDL Solvers performing the actual computations
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  // The variables (which need to be pre-allocated).
  KDL::JntArray  q_;            // Joint positions
  KDL::JntArray  q0_;           // Joint initial positions
  KDL::JntArrayVel  qdot_;      // Joint velocities
  KDL::JntArray  tau_;          // Joint torques

  KDL::JntArray  q_m_;          // Model Joint positions
  KDL::JntArray  qd_m_;         // Model Joint positions
  KDL::JntArray  qdd_m_;        // Model Joint positions

  KDL::Frame     x_;            // Tip pose
  KDL::Frame     xd_;           // Tip desired pose
  KDL::Frame     x0_;           // Tip initial pose

  KDL::Twist     xerr_;         // Cart error
  KDL::Twist     xdot_;         // Cart velocity
  KDL::Wrench    F_;            // Cart effort
  KDL::Twist     ferr_;			// Cart effort error
  KDL::Jacobian  J_;            // Jacobian

  // Note the gains are incorrectly typed as a twist,
  // as there is no appropriate type!
  KDL::Twist     Kp_;           // Proportional gains
  KDL::Twist     Kd_;           // Derivative gains

  // The trajectory variables
  double    circle_phase_;      // Phase along the circle
  ros::Time last_time_;         // Time of the last servo cycle

  //! realtime publisher for max_force value
  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> pub_;
  realtime_tools::RealtimePublisher<sensor_msgs::JointState> pubModelStates_;
  geometry_msgs::WrenchStamped r_forceData;

  //! publish max_force values every X realtime cycles
  int pub_cycle_count_;
  bool should_publish_;

  /////////////////////////
  // System Model
  // Declare the number of joints.
  enum { Joints = 7 };

  // Define the joint/cart vector types accordingly (using a fixed
  // size to avoid dynamic allocations and make the code realtime safe).
  typedef Eigen::Matrix<double, Joints, Joints>  SystemMatrix;
  typedef Eigen::Matrix<double, Joints, 1>		 SystemVector;

  SystemMatrix  Mm;
  SystemMatrix  Dm;
  SystemMatrix  Km;
  SystemMatrix  MmInv;

  SystemVector	q_m;
  SystemVector  qd_m;
  SystemVector 	qdd_m;
  SystemVector 	t_h;

  double delT;
  // System Model END
  /////////////////////////

public:

  bool init(pr2_mechanism_model::RobotState *robot,
            ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();
};
}
