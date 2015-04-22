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
#include <neuroadaptive_msgs/fixedWeightToggle.h>
#include <neuroadaptive_msgs/saveControllerData.h>

//#include "controllerFullData.pb.h"
#include <fstream>

typedef boost::array<double, 4> human_state_type;

namespace pr2_controller_ns{

class PR2CartneuroControllerClass: public pr2_controller_ns::PR2CartesianControllerClass
{
private:

  ros::Subscriber sub_command_;

  /////////////////////////
  // System Model

  csl::outer_loop::MracModel  outerLoopMRACmodelX  ;
  csl::outer_loop::MracModel  outerLoopMRACmodelY  ;

  csl::outer_loop::RlsModel   outerLoopRLSmodelX   ;
  csl::outer_loop::RlsModel   outerLoopRLSmodelY   ;

  csl::outer_loop::MsdModel   outerLoopMSDmodelX   ;
  csl::outer_loop::MsdModel   outerLoopMSDmodelY   ;

  csl::outer_loop::IrlModel   outerLoopIRLmodelX   ;
  csl::outer_loop::IrlModel   outerLoopIRLmodelY   ;

  csl::outer_loop::CtRlsModel outerLoopCTRLSmodelX ;
  csl::outer_loop::CtRlsModel outerLoopCTRLSmodelY ;

  Eigen::MatrixXd outerLoopWk         ;
  Eigen::MatrixXd outerLoopWk_flexi_1 ;
  Eigen::MatrixXd outerLoopWk_flexi_2 ;
  Eigen::MatrixXd outerLoopWk_flexi_3 ;
  Eigen::MatrixXd outerLoopWk_flexi_4 ;

  // Fixed filter weights or adaptive weights
  bool useFixedWeights;

  Eigen::VectorXd q;
  Eigen::VectorXd qd;
  Eigen::VectorXd qdd;

  Eigen::VectorXd q_m;
  Eigen::VectorXd qd_m;
  Eigen::VectorXd qdd_m;

  Eigen::VectorXd X_m;
  Eigen::VectorXd Xd_m;
  Eigen::VectorXd Xdd_m;

  Eigen::VectorXd p_X_m;
  Eigen::VectorXd p_Xd_m;
  Eigen::VectorXd p_Xdd_m;

  Eigen::VectorXd X;
  Eigen::VectorXd Xd;

  Eigen::VectorXd t_r;
  Eigen::VectorXd task_ref;
  Eigen::VectorXd task_refModel_output;
  Eigen::VectorXd tau;
  Eigen::VectorXd force;
  Eigen::VectorXd flexiForce;

  Eigen::MatrixXd Jacobian;

  human_state_type ode_init_x;

  double filtW0 ;   double flex_1_filtW0 ;   double flex_2_filtW0 ;   double flex_3_filtW0 ;   double flex_4_filtW0 ;
  double filtW1 ;   double flex_1_filtW1 ;   double flex_2_filtW1 ;   double flex_3_filtW1 ;   double flex_4_filtW1 ;
  double filtW2 ;   double flex_1_filtW2 ;   double flex_2_filtW2 ;   double flex_3_filtW2 ;   double flex_4_filtW2 ;
  double filtW3 ;   double flex_1_filtW3 ;   double flex_2_filtW3 ;   double flex_3_filtW3 ;   double flex_4_filtW3 ;
  double filtW4 ;   double flex_1_filtW4 ;   double flex_2_filtW4 ;   double flex_3_filtW4 ;   double flex_4_filtW4 ;
  double filtW5 ;   double flex_1_filtW5 ;   double flex_2_filtW5 ;   double flex_3_filtW5 ;   double flex_4_filtW5 ;
  double filtW6 ;   double flex_1_filtW6 ;   double flex_2_filtW6 ;   double flex_3_filtW6 ;   double flex_4_filtW6 ;
  double filtW7 ;   double flex_1_filtW7 ;   double flex_2_filtW7 ;   double flex_3_filtW7 ;   double flex_4_filtW7 ;

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

  double kappa  ;
  double Kv     ;
  double lambda ;
  double Kz     ;
  double Zb     ;
  double nnF    ;
  double nnG    ;
  double nn_ON  ;

  double m_M    ;
  double m_S    ;
  double m_D    ;

  double task_mA ;
  double task_mB ;

  double simHuman_a;
  double simHuman_b;

  double fFForce;

  csl::neural_network::OneLayerNeuralNetworkController nnController;

  // NN END
  /////////////////////////

  double circle_rate         ;
  double circleUlim          ;
  double circleLlim          ;
  bool   startCircleTraj     ;

  bool   externalRefTraj     ;
  bool   directlyUseTaskModel;
  double intentEst_delT      ;
  double intentEst_M         ;

  Eigen::MatrixXd eigen_temp_joint;
  KDL::JntArray kdl_temp_joint_;

  bool setRefTraj( neuroadaptive_msgs::setCartPose::Request  & req ,
                   neuroadaptive_msgs::setCartPose::Response & resp );

  bool paramUpdate( neuroadaptive_msgs::controllerParamUpdate::Request  & req ,
                    neuroadaptive_msgs::controllerParamUpdate::Response & resp );

  bool save( neuroadaptive_msgs::saveControllerData::Request & req,
             neuroadaptive_msgs::saveControllerData::Response& resp );

  bool publish( std_srvs::Empty::Request & req,
                std_srvs::Empty::Response& resp );

  bool capture( std_srvs::Empty::Request& req,
                std_srvs::Empty::Response& resp );

  bool toggleFixedWeights( neuroadaptive_msgs::fixedWeightToggle::Request & req,
		                   neuroadaptive_msgs::fixedWeightToggle::Response& resp );

  ros::ServiceServer save_srv_;
  ros::ServiceServer publish_srv_;
  ros::ServiceServer capture_srv_;
  ros::ServiceServer setRefTraj_srv_;
  ros::ServiceServer toggleFixedWeights_srv_;

  void bufferData( double & dt );
//  void setDataPoint(dataPoint::Datum* datum, double & dt);
//  dataPoint::controllerFullData controllerData;

  std::fstream saveDataFile;

public:
  ~PR2CartneuroControllerClass();
  bool init(pr2_mechanism_model::RobotState *robot,
            ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();

  Eigen::MatrixXd JointKdl2Eigen( KDL::JntArray & joint_ );
  Eigen::MatrixXd JointVelKdl2Eigen( KDL::JntArrayVel & joint_ );
  KDL::JntArray JointEigen2Kdl( Eigen::VectorXd & joint );

  void calcHumanIntentPos( Eigen::Vector3d & force, Eigen::VectorXd & pos, double delT, double m );

  // FIXME change this message type
  void command(const geometry_msgs::WrenchConstPtr& wrench_msg);


};
}
