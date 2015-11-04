
// Utilities
#include <fstream>
#include <boost/scoped_ptr.hpp>
#include "objTest.h"

// PR2
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>
#include <pr2_hardware_interface/hardware_interface.h>

#include "pr2_gripper_sensor_controller/acceleration_observer.h"

// Match
#include <Eigen/Geometry>
#include <angles/angles.h>

#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

// Inner and outer loops
#include "csl/neural_network.hpp"
#include "csl/outer_loop.h"

// ROS messages
#include <ice_msgs/controllerParam.h>
#include <ice_msgs/controllerFullData.h>
#include <ice_msgs/controllerParamUpdate.h>
#include <ice_msgs/saveControllerData.h>
#include <ice_msgs/setCartPose.h>
#include <ice_msgs/fixedWeightToggle.h>

#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <std_srvs/Empty.h>



namespace pr2_controller_ns{

enum
{
	StoreLen = 60000
};

class PR2CartneuroControllerClass: public pr2_controller_interface::Controller //public pr2_controller_ns::PR2CartesianControllerClass
{
private:

	// Definitions
	typedef Eigen::Matrix<double, 7, 1> JointVec;
	typedef boost::array<double, 4> human_state_type;

	// The current robot state (to get the time stamp)
	pr2_mechanism_model::RobotState* robot_state_;

	// The chain of links and joints
	pr2_mechanism_model::Chain chain_;
	pr2_mechanism_model::Chain chain_acc_link;
	KDL::Chain kdl_chain_;
	KDL::Chain kdl_chain_acc_link;

	// KDL Solvers performing the actual computations
	boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_;
	boost::scoped_ptr<KDL::ChainFkSolverPos>    jnt_to_pose_solver_acc_;
	boost::scoped_ptr<KDL::ChainJntToJacSolver> jnt_to_jac_solver_;

	// The variables (which need to be pre-allocated).
	KDL::JntArray  q_;            // Joint positions
	KDL::JntArray  q0_;           // Joint initial positions
	KDL::JntArrayVel  qdot_;      // Joint velocities
	KDL::JntArray  tau_c_;          // Joint torques

	KDL::Frame     x_;            // Tip pose
	KDL::Frame     xd_;           // Tip desired pose
	KDL::Frame     x0_;           // Tip initial pose

	KDL::Frame     x_gripper_acc_;// Gripper accelerometer frame

	KDL::Frame     x_m_;          // Model Tip pose
	KDL::Frame     xd_m_;         // Model Tip desired pose
	KDL::Frame     x0_m_;         // Model Tip initial pose

	KDL::JntArray  qnom;
	KDL::JntArray  q_lower;       // Joint position lower limits
	KDL::JntArray  q_upper;       // Joint position upper limits
	KDL::JntArray  qd_limit;      // Joint velocity limits
	KDL::JntArray  q_nominal;     // Nominal joint angles

	urdf::Model urdf_model;

	KDL::Twist     xerr_;         // Cart error
	KDL::Twist     xdot_;         // Cart velocity
	KDL::Wrench    F_;            // Cart effort
	KDL::Jacobian  J_;            // Jacobian

	JointVec saturation_;         // Saturation torques

	// Note the gains are incorrectly typed as a twist,
	// as there is no appropriate type!
	Eigen::VectorXd     Kp_;           // Proportional gains
	Eigen::VectorXd     Kd_;           // Derivative gains


	// PR2 hardware
	pr2_hardware_interface::Accelerometer* l_accelerometer_handle_;
	pr2_hardware_interface::Accelerometer* r_accelerometer_handle_;

	pr2_hardware_interface::ForceTorque* l_ft_handle_;
	pr2_hardware_interface::ForceTorque* r_ft_handle_;

	accelerationObserver* l_accelerationObserver;
	accelerationObserver* r_accelerationObserver;

	int l_ft_samples;
	int r_ft_samples;

	geometry_msgs::WrenchStamped l_ftBias;
	geometry_msgs::WrenchStamped r_ftBias;

	geometry_msgs::WrenchStamped l_ftData;
	geometry_msgs::WrenchStamped r_ftData;

	Eigen::Vector3d FT_transformed_force ;
	Eigen::Vector3d FLEX_force          ;
	Eigen::Vector3d transformed_force    ;
	Eigen::Vector3d l_acc_data           ;
	Eigen::Vector3d r_acc_data           ;


	// Flexiforce data (input of the controller)
	KDL::Wrench flexiforce_wrench_desi_;


	// Cartesian paramters
	double cartPos_Kp_x ; double cartPos_Kd_x ; // Translation x
	double cartPos_Kp_y ; double cartPos_Kd_y ; // Translation y
	double cartPos_Kp_z ; double cartPos_Kd_z ; // Translation z
	double cartRot_Kp_x ; double cartRot_Kd_x ; // Rotation    x
	double cartRot_Kp_y ; double cartRot_Kd_y ; // Rotation    y
	double cartRot_Kp_z ; double cartRot_Kd_z ; // Rotation    z

	geometry_msgs::Pose modelCartPos_;
	geometry_msgs::Pose robotCartPos_;

	double delT;
	double outerLoopTime;
	double intentLoopTime;

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
	double cartIniRoll  ;
	double cartIniPitch ;
	double cartIniYaw   ;

	Eigen::MatrixXd JacobianPinv;
	Eigen::MatrixXd JacobianTrans;
	Eigen::MatrixXd JacobianTransPinv;
	Eigen::MatrixXd nullSpace;

	Eigen::VectorXd cartControlForce;
	Eigen::VectorXd nullspaceTorque;
	Eigen::VectorXd controlTorque;

	bool useCurrentCartPose ;		// Use current cart pose or use specified cart pose
	bool useNullspacePose ;		// Use nullspace stuff
	bool useFTinput ;				// Use FT info or not

	// Outer loop
	bool useARMAmodel ;			// Use ARMA
	bool useCTARMAmodel ;	  // Use CT ARMA
	bool useFIRmodel ;	  // Use FIR
	bool useMRACmodel ;	  // Use MRAC
	bool useMSDmodel ;	  // Use MSD
	bool useIRLmodel ;	  // Use IRL
	bool useDirectmodel;	  // Use Direct model x_d = x_m

	bool useSimHuman;

	// Use FT sensors or not
	bool forceTorqueOn;
	bool useFlexiForce;

	double forceCutOffX ;
	double forceCutOffY ;
	double forceCutOffZ ;

	// The trajectory variables
	double    circle_phase_;      // Phase along the circle
	ros::Time last_time_;         // Time of the last servo cycle
	ros::Time start_time_;        // Time of the first servo cycle
	ros::Time outer_elapsed_;     // Time elapsed since outer loop call
	ros::Time intent_elapsed_;    // Time elapsed since intent loop call


	// ROS publishers
	ros::Publisher pubFTData_              ;
	ros::Publisher pubModelStates_         ;
	ros::Publisher pubRobotStates_         ;
	ros::Publisher pubModelCartPos_        ;
	ros::Publisher pubRobotCartPos_        ;
	ros::Publisher pubControllerParam_     ;
	ros::Publisher pubControllerFullData_  ;

	// ROS subscribers
	ros::Subscriber sub_command_;

	// ROS messages
	geometry_msgs::WrenchStamped             msgFTData             [StoreLen];
	sensor_msgs::JointState                  msgModelStates        [StoreLen];
	sensor_msgs::JointState                  msgRobotStates        [StoreLen];
	geometry_msgs::PoseStamped               msgModelCartPos       [StoreLen];
	geometry_msgs::PoseStamped               msgRobotCartPos       [StoreLen];
	ice_msgs::controllerParam    msgControllerParam    [StoreLen];
	ice_msgs::controllerFullData msgControllerFullData [StoreLen];

	volatile int storage_index_;


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

	bool setRefTraj( ice_msgs::setCartPose::Request  & req ,
			ice_msgs::setCartPose::Response & resp );

	bool paramUpdate( ice_msgs::controllerParamUpdate::Request  & req ,
			ice_msgs::controllerParamUpdate::Response & resp );

	bool save( ice_msgs::saveControllerData::Request & req,
			ice_msgs::saveControllerData::Response& resp );

	bool publish( std_srvs::Empty::Request & req,
			std_srvs::Empty::Response& resp );

	bool capture( std_srvs::Empty::Request& req,
			std_srvs::Empty::Response& resp );

	bool toggleFixedWeights( ice_msgs::fixedWeightToggle::Request & req,
			ice_msgs::fixedWeightToggle::Response& resp );

	ros::ServiceServer save_srv_;
	ros::ServiceServer publish_srv_;
	ros::ServiceServer capture_srv_;
	ros::ServiceServer setRefTraj_srv_;
	ros::ServiceServer toggleFixedWeights_srv_;

	void bufferData( double & dt );
	//  void setDataPoint(dataPoint::Datum* datum, double & dt);
	//  dataPoint::controllerFullData controllerData;

	std::fstream saveDataFile;
	bool accelerometerOn;


	bool initRobot(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
	bool initTrajectories(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
	bool initInnerLoop(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
	bool initOuterLoop(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
	bool initSensors(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);
	bool initNN(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n);

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


	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
}
