#include "uta_pr2_forceControl/cartesianController.h"
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include "objTest.h"
#include "csl/neural_network.hpp"
#include "csl/outer_loop.h"

typedef boost::array<double, 4> human_state_type;

namespace pr2_controller_ns{

class PR2CartneuroControllerClass: public pr2_controller_ns::PR2CartesianControllerClass
{
private:

  /////////////////////////
  // System Model

//  csl::outer_loop::MsdModel outerLoopMSDmodelJoint1;
//  csl::outer_loop::MsdModel outerLoopMSDmodelJoint2;

  csl::outer_loop::FirModel outerLoopFIRmodelJoint1;
  csl::outer_loop::FirModel outerLoopFIRmodelY;

  csl::outer_loop::MsdModel outerLoopMSDmodelX     ;
  csl::outer_loop::MsdModel outerLoopMSDmodelY     ;

  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd qdd;

  Eigen::VectorXd q_m;
  Eigen::VectorXd qd_m;
  Eigen::VectorXd qdd_m;

  Eigen::VectorXd X_m;
  Eigen::VectorXd Xd_m;
  Eigen::VectorXd Xdd_m;

  Eigen::VectorXd X;
  Eigen::VectorXd Xd;

  Eigen::VectorXd t_r;
  Eigen::VectorXd task_ref;
  Eigen::VectorXd task_refModel;
  Eigen::VectorXd tau;
  Eigen::VectorXd force;

  Eigen::MatrixXd Jacobian;

  human_state_type ode_init_x;

  // System Model END
  /////////////////////////

  /////////////////////////
  // NN

//  enum { Inputs  = 35 }; // n Size of the inputs
//  enum { Outputs = 7 }; // m Size of the outputs
//  enum { Hidden  = 10 }; // l Size of the hidden layer
//  enum { Error   = 7 }; // filtered error

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

  bool paramUpdate( uta_pr2_forceControl::controllerParamUpdate::Request  & req ,
                    uta_pr2_forceControl::controllerParamUpdate::Response & resp );

  bool capture(std_srvs::Empty::Request& req,
               std_srvs::Empty::Response& resp);
  ros::ServiceServer capture_srv_;

  void bufferData( double & dt );

public:
  bool init(pr2_mechanism_model::RobotState *robot,
            ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();

  Eigen::MatrixXd JointKdl2Eigen( KDL::JntArray & joint_ );
  Eigen::MatrixXd JointVelKdl2Eigen( KDL::JntArrayVel & joint_ );
  KDL::JntArray JointEigen2Kdl( Eigen::VectorXd & joint );

};
}
