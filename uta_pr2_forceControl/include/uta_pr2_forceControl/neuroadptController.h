#ifndef NEROADPTCONTROLLER_H_
#define NEROADPTCONTROLLER_H_

#include <pr2_controller_interface/controller.h>
#include <pr2_hardware_interface/hardware_interface.h>
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolver.hpp>
//#include <kdl/chainiksolvervel_pinv_nso.hpp>
#include <kdl/chainiksolvervel_wdls.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include "realtime_tools/realtime_publisher.h"

#include <control_toolbox/pid.h>
#include "control_toolbox/pid_gains_setter.h"

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <neuroadaptive_msgs/controllerParam.h>
#include <neuroadaptive_msgs/controllerFullData.h>
#include <neuroadaptive_msgs/controllerParamUpdate.h>
#include <neuroadaptive_msgs/saveControllerData.h>
#include <neuroadaptive_msgs/setCircleRate.h>

#include <std_srvs/Empty.h>

#include <Eigen/StdVector>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "ros/ros.h"
#include <urdf/model.h>

#include <boost/array.hpp>
#include <boost/numeric/odeint.hpp>

#include "csl/neural_network.hpp"
//#include "csl/outer_loop.h"

typedef boost::array< double , 21 > state_type;
typedef boost::array< double , 4 > state_type_4;
typedef boost::array< double , 6 > joint_type_6;

namespace pr2_controller_ns{

enum
{
  StoreLen = 10000
};

class PR2NeuroadptControllerClass: public pr2_controller_interface::Controller
{
protected:
  // The current robot state (to get the time stamp)
  pr2_mechanism_model::RobotState* robot_state_;

  // The chain of links and joints
  pr2_mechanism_model::Chain chain_;
  KDL::Chain kdl_chain_;

  pr2_hardware_interface::AnalogIn* analogin_handle_;
  pr2_hardware_interface::Accelerometer* accelerometer_handle_;

  pr2_hardware_interface::ForceTorque* l_ft_handle_;
  pr2_hardware_interface::ForceTorque* ft_handle_;

  int l_ft_samples;
  int r_ft_samples;

  geometry_msgs::WrenchStamped l_ftBias;
  geometry_msgs::WrenchStamped r_ftBias;

  geometry_msgs::WrenchStamped l_ftData;
  geometry_msgs::WrenchStamped ftData;

  Eigen::Vector3d transformed_force    ;

  sensor_msgs::JointState modelState;
  sensor_msgs::JointState robotState;

  // KDL Solvers performing the actual computations
  boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
  boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_ ;

//  boost::scoped_ptr<KDL::ChainIkSolverPos_LMA>    pos_to_jnt_solver_ ;
  boost::scoped_ptr<KDL::ChainIkSolverVel_wdls>   vel_to_jnt_solver_ ;
//  boost::scoped_ptr<KDL::ChainIkSolverAcc>    acc_to_jnt_solver_ ;

  // The variables (which need to be pre-allocated).
  KDL::JntArray     q_;            // Joint positions
  KDL::JntArray     q0_;           // Joint initial positions
  KDL::JntArrayVel  qdot_;         // Joint velocities
  KDL::JntArray     tau_t_;         // Joint torques from trajectory/impedance
  KDL::JntArray     tau_h_;         // Joint torques from human
  KDL::JntArray     tau_c_;         // Joint command joint torques
  KDL::JntArray     tau_f_;         // Joint torques from robot feedback

  KDL::JntArray     q_m_;          // Model Joint positions
  KDL::JntArray     q0_m_;         // Joint initial positions
  KDL::JntArray     qd_m_;         // Model Joint positions
  KDL::JntArray     qdd_m_;        // Model Joint positions

  KDL::JntArray     q_lower;       // Joint position lower limits
  KDL::JntArray     q_upper;       // Joint position upper limits
  KDL::JntArray     qd_limit;      // Joint velocity limits
  KDL::JntArray     q_nominal;     // Nominal joint angles

  KDL::Frame        x_;            // Robot Tip pose
  KDL::Frame        xd_;           // Robot Tip desired pose
  KDL::Frame        x0_;           // Robot Tip initial pose

  KDL::Frame        x_m_;          // Model Tip pose
  KDL::Frame        xd_m_;         // Model Tip desired pose
  KDL::Frame        x0_m_;         // Model Tip initial pose

  KDL::Twist        xerr_;         // Cart error
  KDL::Twist        xdot_;         // Cart velocity
  KDL::Wrench       F_;            // Cart effort
  KDL::Twist        ferr_;	   // Cart effort error

  KDL::Jacobian     J_;            // Robot Jacobian
  KDL::Jacobian     J_m_;          // Model Jacobian

  double forceCutOffX ;
  double forceCutOffY ;
  double forceCutOffZ ;

  // Note the gains are incorrectly typed as a twist,
  // as there is no appropriate type!
  KDL::Twist        Kp_;           // Proportional gains
  KDL::Twist        Kd_;           // Derivative gains

  // The trajectory variables
  double    circle_phase_;      // Phase along the circle
  ros::Time last_time_ ;        // Time of the last servo cycle
  ros::Time start_time_;        // Time of the first servo cycle

  std::vector<control_toolbox::Pid> jointPid;

  geometry_msgs::WrenchStamped r_forceData;
  geometry_msgs::Pose modelCartPos_;
  geometry_msgs::Pose robotCartPos_;

  //! publish max_force values every X realtime cycles
  int pub_cycle_count_;
  bool should_publish_;

  /////////////////////////
  // System Model

  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd qdd;

  Eigen::VectorXd q_m;
  Eigen::VectorXd qd_m;
  Eigen::VectorXd qdd_m;

  Eigen::VectorXd x_m;
  Eigen::VectorXd xd_m;
  Eigen::VectorXd xdd_m;

  Eigen::VectorXd t_r;
  Eigen::VectorXd task_ref;
  Eigen::VectorXd tau;

  Eigen::VectorXd joint_P;
  Eigen::VectorXd joint_I;
  Eigen::VectorXd joint_D;
  Eigen::VectorXd joint_Imax;
  Eigen::VectorXd joint_Imin;

  KDL::JntArray  kdl_qmdot_;
  KDL::Twist     kdl_xd_m_;

  double delT;

  state_type_4 vpol_init_x;
  joint_type_6 joint_vel_qd;

  // System Model END
  /////////////////////////

  /////////////////////////
  // NN

  double num_Inputs  ; // n Size of the inputs
  double num_Outputs ; // m Size of the outputs
  double num_Hidden  ; // l Size of the hidden layer
  double num_Error   ; // filtered error
  double num_Joints  ; // number of joints.

  double  kappa  ;
  double  Kv     ;
  double  lambda ;
  double  Kz     ;
  double  Zb     ;
  double  nnF    ;
  double  nnG    ;
  double  nn_ON  ;

  double  m_M    ;
  double  m_S    ;
  double  m_D    ;

  double  fFForce;

  csl::neural_network::TwoLayerNeuralNetworkController nnController;

  // NN END
  /////////////////////////

  double circle_rate;
  double circleUlim ;
  double circleLlim ;
  bool startCircleTraj;
  Eigen::MatrixXd eigen_temp_joint;
  KDL::JntArray kdl_temp_joint_;

  urdf::Model urdf_model;

  bool paramUpdate( neuroadaptive_msgs::controllerParamUpdate::Request  & req ,
                    neuroadaptive_msgs::controllerParamUpdate::Response & resp );

  bool circleRateCB( neuroadaptive_msgs::setCircleRate::Request & req,
  	                 neuroadaptive_msgs::setCircleRate::Response& resp );

  bool save( neuroadaptive_msgs::saveControllerData::Request & req,
  	         neuroadaptive_msgs::saveControllerData::Response& resp );

  bool publish( std_srvs::Empty::Request & req,
                std_srvs::Empty::Response& resp );

  bool capture(std_srvs::Empty::Request& req,
               std_srvs::Empty::Response& resp);

  bool saveControllerData( neuroadaptive_msgs::saveControllerData::Request&  req,
                           neuroadaptive_msgs::saveControllerData::Response& resp );

  ros::ServiceServer set_circle_rate_srv_;
  ros::ServiceServer save_srv_;
  ros::ServiceServer publish_srv_;
  ros::ServiceServer capture_srv_;
  ros::ServiceServer paramUpdate_srv_;

  void bufferData( double & dt );
  Eigen::MatrixXd JointKdl2Eigen( KDL::JntArray & joint_ );
  Eigen::MatrixXd JointVelKdl2Eigen( KDL::JntArrayVel & joint_ );
  KDL::JntArray JointEigen2Kdl( Eigen::VectorXd & joint );

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
  neuroadaptive_msgs::controllerParam      msgControllerParam    [StoreLen];
  neuroadaptive_msgs::controllerFullData   msgControllerFullData [StoreLen];

  volatile int storage_index_;

public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  bool init(pr2_mechanism_model::RobotState *robot,
            ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();

};
}

#endif /* NEROADPTCONTROLLER_H_ */
