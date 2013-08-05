#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

//#include <pr2_mechanism_model/kinematichelpers.h>
#include <Eigen/Geometry>


namespace pr2_controller_ns{

class PR2CartControllerClass: public pr2_controller_interface::Controller
{
  // Declare the number of joints.
  enum
  {
    Joints = 7
  };

  // Define the joint/cart vector types accordingly (using a fixed
  // size to avoid dynamic allocations and make the code realtime safe).
  typedef Eigen::Matrix<double, Joints, 1>  JointVector;
  typedef Eigen::Transform3d                CartPose;
  typedef Eigen::Matrix<double, 3, 1>       Cart3Vector;
  typedef Eigen::Matrix<double, 6, 1>       Cart6Vector;
  typedef Eigen::Matrix<double, 6, Joints>  JacobianMatrix;

  // Ensure 128-bit alignment for Eigen
  // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 private:
  // The current robot state (to get the time stamp)
  pr2_mechanism_model::RobotState* robot_state_;

  // The chain of links and joints
  pr2_mechanism_model::Chain chain_;

  // KDL Solvers performing the actual computations
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  // Temporary variables for KDL and helper functions
  // to convert to Eigen, etc.
  KDL::JntArray qtmp_;          // Joint vector
  KDL::Frame    xtmp_;          // Tip pose
  KDL::Jacobian Jtmp_;          // Jacobian
  //pr2_mechanism_model::KinematicHelpers kin_;
 
  // The trajectory initial value
  CartPose  x0_;                // Tip initial pose

  // The controller parameters
  Cart6Vector  Kp_;             // Proportional gains
  Cart6Vector  Kd_;             // Derivative gains

  // The trajectory variables
  double    circle_phase_;      // Phase along the circle
  ros::Time last_time_;         // Time of the last servo cycle

 public:
  bool init(pr2_mechanism_model::RobotState *robot,
            ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();
};
}
