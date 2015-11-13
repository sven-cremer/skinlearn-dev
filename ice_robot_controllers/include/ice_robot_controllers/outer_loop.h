// Utilities
#include <fstream>
#include <boost/scoped_ptr.hpp>
#include "objTest.h"
#include <ice_robot_controllers/KDLcontroller.h>

// Inner and outer loops
#include "csl/neural_network.hpp"
#include "csl/outer_loop.h"

//PR2
#include <pr2_mechanism_model/chain.h>

//KDL
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

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

#include <ros/ros.h>

typedef boost::array<double, 4> human_state_type;
typedef Eigen::Matrix<double, 7, 1> JointVec;

class outer_loop{
private:
	ros::NodeHandle nh_;

	bool useARMAmodel ;			// Use ARMA
	bool useCTARMAmodel ;	  	// Use CT ARMA
	bool useFIRmodel ;	  		// Use FIR
	bool useMRACmodel ;	  		// Use MRAC
	bool useMSDmodel ;	  		// Use MSD
	bool useIRLmodel ;	  		// Use IRL
	bool useDirectmodel;	  	// Use Direct model x_d = x_m

	bool useSimHuman;

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

	human_state_type ode_init_x;

	double filtW0 ;   double flex_1_filtW0 ;   double flex_2_filtW0 ;   double flex_3_filtW0 ;   double flex_4_filtW0 ;
	double filtW1 ;   double flex_1_filtW1 ;   double flex_2_filtW1 ;   double flex_3_filtW1 ;   double flex_4_filtW1 ;
	double filtW2 ;   double flex_1_filtW2 ;   double flex_2_filtW2 ;   double flex_3_filtW2 ;   double flex_4_filtW2 ;
	double filtW3 ;   double flex_1_filtW3 ;   double flex_2_filtW3 ;   double flex_3_filtW3 ;   double flex_4_filtW3 ;
	double filtW4 ;   double flex_1_filtW4 ;   double flex_2_filtW4 ;   double flex_3_filtW4 ;   double flex_4_filtW4 ;
	double filtW5 ;   double flex_1_filtW5 ;   double flex_2_filtW5 ;   double flex_3_filtW5 ;   double flex_4_filtW5 ;
	double filtW6 ;   double flex_1_filtW6 ;   double flex_2_filtW6 ;   double flex_3_filtW6 ;   double flex_4_filtW6 ;
	double filtW7 ;   double flex_1_filtW7 ;   double flex_2_filtW7 ;   double flex_3_filtW7 ;   double flex_4_filtW7 ;

	//System Models
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

	//Default parameter
	int numIrlSamples;
	int numIrlLsIter;
	int numCartDof;
	bool irlOneshot;
	double rls_lambda;
	double rls_sigma;

	//?
	bool useCurrentCartPose ;		// Use current cart pose or use specified cart pose
	bool useNullspacePose ;		// Use nullspace stuff
	bool useFTinput ;				// Use FT info or not

	double m_M    ;
	double m_S    ;
	double m_D    ;

	double mrac_gamma_1;
	double mrac_gamma_2;
	double mrac_gamma_3;
	double mrac_gamma_4;
	double mrac_gamma_5;

	double mrac_P_m;
	double mrac_P_h;

	double delT;
	double outerLoopTime;
	double intentLoopTime;

	double simHuman_a;
	double simHuman_b;

	double task_mA ;
	double task_mB ;

	bool externalRefTraj;
	bool   directlyUseTaskModel;
	double intentEst_delT      ;
	double intentEst_M         ;

	double forceCutOffX ;
	double forceCutOffY ;
	double forceCutOffZ ;

	// Initial cartesian pose
	double cartIniX ;
	double cartIniY ;
	double cartIniZ ;
	double cartIniRoll  ;
	double cartIniPitch ;
	double cartIniYaw   ;

	Eigen::MatrixXd Jacobian;
	Eigen::MatrixXd JacobianPinv;
	Eigen::MatrixXd JacobianTrans;
	Eigen::MatrixXd JacobianTransPinv;
	Eigen::MatrixXd nullSpace;

	Eigen::VectorXd cartControlForce;
	Eigen::VectorXd nullspaceTorque;
	Eigen::VectorXd controlTorque;

	/////////////////////////
	// NN
	double num_Inputs  ; // n Size of the inputs
	double num_Outputs ; // m Size of the outputs
	double num_Hidden  ; // l Size of the hidden layer
	double num_Error   ; // filtered error
	double num_Joints  ; // number of joints.

	// The chain of links and joints
	pr2_mechanism_model::Chain chain_;

	JointVec saturation_;         // Saturation torques

	Eigen::MatrixXd eigen_temp_joint;
	KDL::JntArray kdl_temp_joint_;

	Eigen::Vector3d transformed_force    ;
	Eigen::Vector3d l_acc_data           ;
	Eigen::Vector3d r_acc_data           ;

	KDL::Chain kdl_chain_;


	//Added new members. Not sure if we should initialize them in this class or initialize them outside them and use a setter to use them in this class
	// The current robot state (to get the time stamp)
	pr2_mechanism_model::RobotState* robot_state_;	// The current robot state (to get the time stamp)
	ros::Time outer_elapsed_;     					// Time elapsed since outer loop call
	ros::Time intent_elapsed_;    					// Time elapsed since intent loop call
	KDL::Frame     x_gripper_acc_;					// Gripper accelerometer frame
	bool useFlexiForce;

public:
	outer_loop();
	~outer_loop();
	bool load_parameters();
	void update();
	void calcHumanIntentPos( Eigen::Vector3d & force,
			Eigen::VectorXd & pos,
			double delT,
			double m );
};
