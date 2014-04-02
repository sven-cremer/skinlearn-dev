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

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"

#include <uta_pr2_forceControl/controllerParam.h>
#include <uta_pr2_forceControl/controllerFullData.h>
#include <uta_pr2_forceControl/controllerParamUpdate.h>
#include <uta_pr2_forceControl/saveControllerData.h>
#include <uta_pr2_forceControl/setCartPose.h>

#include <Eigen/Geometry>

#include <std_srvs/Empty.h>

#include <angles/angles.h>

namespace pr2_controller_ns{

enum
{
  StoreLen = 10000
};

class PR2CartesianControllerClass: public pr2_controller_interface::Controller
{
public:
  // FIXME remove after testing
  typedef Eigen::Matrix<double, 7, 1> JointVec;

  // The current robot state (to get the time stamp)
  pr2_mechanism_model::RobotState* robot_state_;

  // The chain of links and joints
  pr2_mechanism_model::Chain chain_;
  KDL::Chain kdl_chain_;

  pr2_hardware_interface::Accelerometer* accelerometer_handle_;

  pr2_hardware_interface::ForceTorque* l_ft_handle_;
  pr2_hardware_interface::ForceTorque* r_ft_handle_;

  int l_ft_samples;
  int r_ft_samples;

  geometry_msgs::WrenchStamped l_ftBias;
  geometry_msgs::WrenchStamped r_ftBias;

  geometry_msgs::WrenchStamped l_ftData;
  geometry_msgs::WrenchStamped r_ftData;

  Eigen::Vector3d transformed_force    ;

  // KDL Solvers performing the actual computations
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

  // The variables (which need to be pre-allocated).
  KDL::JntArray  q_;            // Joint positions
  KDL::JntArray  q0_;           // Joint initial positions
  KDL::JntArrayVel  qdot_;      // Joint velocities
  KDL::JntArray  tau_c_;          // Joint torques

  KDL::Frame     x_;            // Tip pose
  KDL::Frame     xd_;           // Tip desired pose
  KDL::Frame     x0_;           // Tip initial pose

  KDL::Frame     x_m_;          // Model Tip pose
  KDL::Frame     xd_m_;         // Model Tip desired pose
  KDL::Frame     x0_m_;         // Model Tip initial pose

  KDL::JntArray  qnom;
  KDL::JntArray  q_lower;       // Joint position lower limits
  KDL::JntArray  q_upper;       // Joint position upper limits
  KDL::JntArray  qd_limit;      // Joint velocity limits

  urdf::Model urdf_model;

  KDL::Twist     xerr_;         // Cart error
  KDL::Twist     xdot_;         // Cart velocity
  KDL::Wrench    F_;            // Cart effort
  KDL::Jacobian  J_;            // Jacobian

  // Note the gains are incorrectly typed as a twist,
  // as there is no appropriate type!
  Eigen::VectorXd     Kp_;           // Proportional gains
  Eigen::VectorXd     Kd_;           // Derivative gains

  double cartPos_Kp_x ; double cartPos_Kd_x ; // Translation x
  double cartPos_Kp_y ; double cartPos_Kd_y ; // Translation y
  double cartPos_Kp_z ; double cartPos_Kd_z ; // Translation z
  double cartRot_Kp_x ; double cartRot_Kd_x ; // Rotation    x
  double cartRot_Kp_y ; double cartRot_Kd_y ; // Rotation    y
  double cartRot_Kp_z ; double cartRot_Kd_z ; // Rotation    z

  geometry_msgs::Pose modelCartPos_;
  geometry_msgs::Pose robotCartPos_;

  double delT;

  // Desired cartesian pose
  double cartDesX     ;
  double cartDesY     ;
  double cartDesZ     ;
  double cartDesRoll  ;
  double cartDesPitch ;
  double cartDesYaw   ;

  // Initial cartesian pose
  double cartIniX ;
  double cartIniY ;
  double cartIniZ ;

  Eigen::MatrixXd Jacobian;
  Eigen::MatrixXd JacobianPinv;
  Eigen::MatrixXd JacobianTrans;
  Eigen::MatrixXd JacobianTransPinv;
  Eigen::MatrixXd nullSpace;

  Eigen::VectorXd cartControlForce;
  Eigen::VectorXd nullspaceTorque;
  Eigen::VectorXd controlTorque;

  // Use current cart pose or use specifiec cart pose
  bool useCurrentCartPose ;

  // Use nullspace stuff
  bool useNullspacePose ;


  // The trajectory variables
  double    circle_phase_;      // Phase along the circle
  ros::Time last_time_;         // Time of the last servo cycle
  ros::Time start_time_;        // Time of the first servo cycle

  double circleUlim ;
  double circleLlim ;
  bool startCircleTraj;
  Eigen::MatrixXd eigen_temp_joint;
  KDL::JntArray kdl_temp_joint_;

  bool paramUpdate( uta_pr2_forceControl::controllerParamUpdate::Request  & req ,
                    uta_pr2_forceControl::controllerParamUpdate::Response & resp );

  bool capture(std_srvs::Empty::Request& req,
               std_srvs::Empty::Response& resp);

  ros::ServiceServer capture_srv_;

  void bufferData( double & dt );

  ros::Publisher pubFTData_              ;
  ros::Publisher pubModelStates_         ;
  ros::Publisher pubRobotStates_         ;
  ros::Publisher pubModelCartPos_        ;
  ros::Publisher pubRobotCartPos_        ;
  ros::Publisher pubControllerParam_     ;
  ros::Publisher pubControllerFullData_  ;

  geometry_msgs::WrenchStamped             msgFTData             [StoreLen];
  sensor_msgs::JointState                  msgModelStates        [StoreLen];
  sensor_msgs::JointState                  msgRobotStates        [StoreLen];
  geometry_msgs::PoseStamped               msgModelCartPos       [StoreLen];
  geometry_msgs::PoseStamped               msgRobotCartPos       [StoreLen];
  uta_pr2_forceControl::controllerParam    msgControllerParam    [StoreLen];
  uta_pr2_forceControl::controllerFullData msgControllerFullData [StoreLen];

  volatile int storage_index_;

public:
  bool init(pr2_mechanism_model::RobotState *robot,
            ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

};
}
