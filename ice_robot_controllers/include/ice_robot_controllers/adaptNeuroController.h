
// Utilities
#include <boost/scoped_ptr.hpp>
#include <boost/thread.hpp>
//#include "objTest.h"
#include <ice_robot_controllers/KDLcontroller.h>
#include <ice_robot_controllers/EigenConversions.h>
#include <realtime_tools/realtime_publisher.h>

// PR2
#include <pr2_controller_interface/controller.h>
#include <pr2_mechanism_model/chain.h>
#include <pr2_mechanism_model/robot.h>
#include <pr2_hardware_interface/hardware_interface.h>

//#include "pr2_gripper_sensor_controller/acceleration_observer.h"
//#include <pr2_gripper_sensor_controller/digitalFilter.h>
#include <ice_robot_controllers/digitalFilter.h>

// Math
#include <Eigen/Geometry>
#include <angles/angles.h>

#include <kdl/chain.hpp>
//#include <kdl/chainjnttojacsolver.hpp>
//#include <kdl/chainfksolverpos_recursive.hpp>
//#include <kdl/frames.hpp>
//#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

// Inner and outer loops
//#include "csl/neural_network.hpp"
#include "csl/nn_controller.cpp"
#include "csl/outer_loop.h"

// ROS messages
#include <ice_msgs/neuroControllerState.h>
#include <ice_msgs/controllerParam.h>
#include <ice_msgs/controllerFullData.h>
#include <ice_msgs/controllerParamUpdate.h>
#include <ice_msgs/saveControllerData.h>
#include <ice_msgs/setCartPose.h>
#include <ice_msgs/fixedWeightToggle.h>
#include <ice_msgs/setBool.h>
#include <ice_msgs/getNNweights.h>
#include <ice_msgs/setNNweights.h>
#include <ice_msgs/setValue.h>
#include <ice_msgs/setInteger.h>
#include <ice_msgs/experimentDataA.h>
#include <ice_msgs/experimentDataB.h>
#include <ice_msgs/experimentDataC.h>
#include <ice_msgs/tactileArrayData.h>
#include <ice_msgs/tactileCalibration.h>
#include <ice_msgs/tactileFilterWeights.h>
#include <ice_msgs/setTrajectory.h>

#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/WrenchStamped.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <std_srvs/Empty.h>



namespace pr2_controller_ns{

class PR2adaptNeuroControllerClass: public pr2_controller_interface::Controller
{
public:
	// Ensure 128-bit alignment for Eigen
	// See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
	enum { StoreLen = 20000 };
	enum { Joints = 7 };
	enum Experiment{ A, B, C, D, Done };

	// Definitions
	typedef Eigen::Matrix<double, 7, 1> JointVec;
	typedef Eigen::Matrix<double, 6, 1> CartVec;
	typedef Eigen::Matrix<double, 6, Joints> JacobianMat;
	typedef boost::array<double, 4> human_state_type;
	typedef ice_msgs::neuroControllerState StateMsg;

	int loopRateFactor;

	ros::NodeHandle nh_;
	double dt_;				// Servo loop time step

	// The current robot state (to get the time stamp)
	pr2_mechanism_model::RobotState* robot_state_;

	urdf::Model urdf_model;

	// The chain of links and joints
	std::string root_name;
	std::string tip_name;
	std::string arm_prefix;			// Either l (left) or r (right)
	pr2_mechanism_model::Chain chain_;
	pr2_mechanism_model::Chain chain_acc_link;
	pr2_mechanism_model::Chain chain_ft_link;
	pr2_mechanism_model::Chain chain_acc_to_ft_link;

	KDL::Chain kdl_chain_;
	KDL::Chain kdl_chain_acc_link;
	KDL::Chain kdl_chain_ft_link;
	KDL::Chain kdl_chain_acc_to_ft_link;

	// KDL Solvers
	boost::scoped_ptr<Kin<Joints> > kin_;
	boost::scoped_ptr<Kin<Joints> > kin_acc_;
	boost::scoped_ptr<Kin<Joints> > kin_ft_;
	boost::scoped_ptr<Kin<Joints> > kin_acc_to_ft_;

	// The variables (which need to be pre-allocated).
	JointVec q_;					// Joint positions
	JointVec q0_;					// Joint initial positions
	JointVec qdot_;					// Joint velocities
	JointVec qdot_raw_;
	JointVec qdot_filtered_;
	double joint_vel_filter_;

	KDL::JntArray tau_measured_;
	CartVec force_measured_;

	JointVec tau_;					// Joint torques
	KDL::JntArray  tau_c_;      	// Joint control torques
	KDL::JntArray  tau_c_latest_;   // Latest joint control torques
	//JointVec tau_posture;			// Joint posture torques
	JointVec q_posture_;

	Eigen::Affine3d x_;				// Tip pose
	Eigen::Affine3d x0_;			// Tip initial pose
	Eigen::Affine3d x0_cali_;		// Tip initial pose for calibration
	CartVec x0_cali_vec_;

	Eigen::Affine3d x_des_;			// Tip desired pose
	Eigen::Affine3d xd_des_;		// Tip desired velocity
	Eigen::Affine3d xdd_des_;		// Tip desired acceleration

	Eigen::Affine3d x_acc_;			// Gripper accelerometer pose
	Eigen::Affine3d x_ft_;			// FT pose
	Eigen::Affine3d x_acc_to_ft_;	// Acc to FT transform

	Eigen::Matrix3d W_mat_;			// Force transformation matrix
	Eigen::Vector3d forceFT;		// FT force vector
	Eigen::Vector3d tauFT;			// FT torque vector

	CartVec ft_bias;				// FT bias
	double gripper_mass;			// Gripper mass
	Eigen::Vector3d r_gripper_com;	// Gripper Center Of Mass pose in FT frame

//	KDL::Frame     x_m_;          // Model Tip pose
//	KDL::Frame     xd_m_;         // Model Tip desired pose
//	KDL::Frame     x0_m_;         // Model Tip initial pose

	KDL::JntArray  qnom;
	KDL::JntArray  q_lower;       // Joint position lower limits
	KDL::JntArray  q_upper;       // Joint position upper limits
	KDL::JntArray  qd_limit;      // Joint velocity limits
	KDL::JntArray  q_nominal;     // Nominal joint angles

	CartVec xerr_;					// Cartesian error
	CartVec xdot_;					// Cartesian velocity
	JacobianMat J_;					// Jacobian
	JacobianMat J_ft_;				// Jacobian FT sensor
	JacobianMat J_acc_;				// Jacobian Acc sensor

	JointVec saturation_;			// Saturation torques

	// Note the gains are incorrectly typed as a twist,
	// as there is no appropriate type!
	Eigen::VectorXd     Kp_;           // Proportional gains
	Eigen::VectorXd     Kd_;           // Derivative gains

	// PR2 hardware
	pr2_hardware_interface::ForceTorque* ft_handle_;
	pr2_hardware_interface::Accelerometer* accelerometer_handle_;

	// Accelerometer
	bool accData_received;
	std::vector<geometry_msgs::Vector3> accData_vector;
	int accData_vector_size;
	Eigen::Vector3d accData;

	// Force/Torque
	bool ftData_received;
	std::vector<geometry_msgs::Wrench> ftData_vector;
	int ftData_vector_size;
	geometry_msgs::WrenchStamped ftData_msg;

	Eigen::Vector3d FT_transformed_force ;
	Eigen::Vector3d transformed_force    ;

	// Use FT sensors or not
	bool forceTorqueOn;
	bool useFlexiForce;
	bool calibrateSensors;
	std::string ft_frame_id;
	CartVec wrench_raw_;
	CartVec wrench_compensated_;
	CartVec wrench_transformed_;
	CartVec wrench_gripper_;			// Gripper wrench

	double forceCutOffX ;
	double forceCutOffY ;
	double forceCutOffZ ;

	bool useDigitalFilter;			// Flag for using filter
    CartVec wrench_filtered_;		// Filtered data
	std::vector<digitalFilter> digitalFilters;	// Filter object
	Eigen::VectorXd a_filt;			// Filter coefficients (denominator)
	Eigen::VectorXd b_filt;			// Filter coefficients (numerator)

	// Flexiforce data (input of the controller)
	CartVec tactile_wrench_;

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
	double outer_delT;
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

	// Nullspace
	Eigen::MatrixXd JacobianPinv;
	Eigen::MatrixXd JacobianTrans;
	Eigen::MatrixXd JacobianTransPinv;
	Eigen::MatrixXd nullSpace;

	// Computes pseudo-inverse of J
	Eigen::Matrix<double,6,6> IdentityCart;
	Eigen::Matrix<double,6,6> JJt_damped;
	Eigen::Matrix<double,6,6> JJt_inv_damped;
	Eigen::Matrix<double,Joints,6> J_pinv;
	Eigen::Matrix<double,Joints,Joints> IdentityJoints;

	// Posture control
	double k_posture;
	double jacobian_inverse_damping;
	JointVec joint_dd_ff_;
	JointVec q_jointLimit ;

	Eigen::VectorXd cartControlForce;
	Eigen::VectorXd nullspaceTorque;
	Eigen::VectorXd controlTorque;

	bool useCurrentCartPose ;		// Use current cart pose or use specified cart pose
	bool useNullspacePose ;		// Use nullspace stuff

	// Torque Saturation
	double sat_scaling;
	Eigen::VectorXd tau_sat;

	// Outer loop
	bool useOuterloop ;
	bool useARMAmodel ;		// Use ARMA
	bool useCTARMAmodel ;	// Use CT ARMA
	bool useFIRmodel ;		// Use FIR
	bool useMRACmodel ;		// Use MRAC
	bool useMSDmodel ;		// Use MSD
	bool useIRLmodel ;		// Use IRL
	bool useDirectmodel;	// Use Direct model x_d = x_m

	bool useSimHuman;

	// The trajectory variables
	ros::Time last_time_;			// Time of the last servo cycle
	ros::Time start_time_;			// Time of the first servo cycle
	ros::Time outer_elapsed_;		// Time elapsed since outer loop call
	ros::Time intent_elapsed_;		// Time elapsed since intent loop call

	// ROS publishers
	ros::Publisher pubFTData_              ;
	ros::Publisher pubModelStates_         ;
	ros::Publisher pubRobotStates_         ;
	ros::Publisher pubModelCartPos_        ;
	ros::Publisher pubRobotCartPos_        ;
	ros::Publisher pubControllerParam_     ;
	ros::Publisher pubControllerFullData_  ;
	ros::Publisher pubExperimentDataA_     ;
	ros::Publisher pubExperimentDataB_     ;
	ros::Publisher pubExperimentDataC_     ;

	bool publishRTtopics;

	// ROS subscribers
	ros::Subscriber sub_tactileWrench_;
	ros::Subscriber sub_tactileData_  ;

	// ROS messages
	geometry_msgs::WrenchStamped msgFTData             [StoreLen];
	sensor_msgs::JointState      msgModelStates        [StoreLen];
	sensor_msgs::JointState      msgRobotStates        [StoreLen];
	geometry_msgs::PoseStamped   msgModelCartPos       [StoreLen];
	geometry_msgs::PoseStamped   msgRobotCartPos       [StoreLen];
	ice_msgs::controllerParam    msgControllerParam    [StoreLen];
	ice_msgs::controllerFullData msgControllerFullData [StoreLen];
	ice_msgs::experimentDataA    experimentDataA_msg_  [StoreLen];
	ice_msgs::experimentDataB    experimentDataB_msg_  [StoreLen];
	ice_msgs::experimentDataC    experimentDataC_msg_  [StoreLen];

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
	Eigen::MatrixXd outerLoopWk_flexi_0 ;
	Eigen::MatrixXd outerLoopWk_flexi_1 ;
	Eigen::MatrixXd outerLoopWk_flexi_2 ;
	Eigen::MatrixXd outerLoopWk_flexi_3 ;

	std::vector<csl::outer_loop::RlsModel*> ARMAmodel_flexi_;
	Eigen::MatrixXd filterWeights_flexi_;
	Eigen::VectorXd tactile_data_;
	int tactile_dimensions_;
	Eigen::MatrixXd sensorDirections;
	int calibrationCounter;
	double calibrationDistance_;
	double calibrationVelocity_;
	double maxCalibrationDistance_;
	bool refTrajSetForCalibration;
	bool calibrateSingelSensors;
	csl::outer_loop::RlsModel* ARMAmodel_flexi_combined_;

	Eigen::MatrixXd weightsRLSmodelX ;
	Eigen::MatrixXd weightsRLSmodelY ;

	// Fixed filter weights or adaptive weights
	bool useFixedWeights;

	Eigen::Vector3d Vec3d_ones;

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

	Eigen::VectorXd x_d;
	Eigen::VectorXd prev_x_d;

	Eigen::VectorXd t_r;
	Eigen::VectorXd task_ref;
	Eigen::VectorXd x_r;
	Eigen::VectorXd xd_r;
	Eigen::VectorXd xdd_r;
	Eigen::VectorXd prev_x_r;
	double delta_x;
	Eigen::VectorXd task_refModel_output;
	//Eigen::VectorXd tau;
	Eigen::VectorXd force_c;
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

	//csl::neural_network::OneLayerNeuralNetworkController nnController;			// TODO make easier to change
	//csl::neural_network::TwoLayerNeuralNetworkController nnController;
	//csl::neural_network::RBFNeuralNetworkController nnController;
	//csl::neural_network::RBFNeuralNetworkController rbfnnController;
	csl::neural_network::NNController* ptrNNController;

	// NN END
	/////////////////////////

	bool mannequinMode;
	double mannequinThresPos;
	double mannequinThresRot;

	double circle_phase        ;	// Phase along the circle
	double circle_rate         ;	// Angular velocity
	double circle_velocity     ;	// Linear velocity
	double circleUlim          ;
	double circleLlim          ;
	double circleAmpl          ;
	bool   startCircleTraj     ;
	int    loopsCircleTraj     ;

	bool   externalRefTraj     ;
	bool   directlyUseTaskModel;
	double intentEst_delT      ;
	double intentEst_M         ;
	bool   useHumanIntent      ;

	bool setRefTraj( ice_msgs::setCartPose::Request  & req ,
			           ice_msgs::setCartPose::Response & resp );

	bool paramUpdate( ice_msgs::controllerParamUpdate::Request  & req ,
			            ice_msgs::controllerParamUpdate::Response & resp );

	bool publishExperimentData( std_srvs::Empty::Request & req,
			        std_srvs::Empty::Response& resp );

	bool capture( std_srvs::Empty::Request& req,
			        std_srvs::Empty::Response& resp );

	bool toggleFixedWeights( ice_msgs::fixedWeightToggle::Request & req,
			                    ice_msgs::fixedWeightToggle::Response& resp );

	bool updateInnerNNweights( ice_msgs::setBool::Request& req,
			              ice_msgs::setBool::Response& resp );
	bool updateNNweights( ice_msgs::setBool::Request& req,
			              ice_msgs::setBool::Response& resp );
	bool setNNweights(    ice_msgs::setNNweights::Request& req,
			              ice_msgs::setNNweights::Response& resp );
	bool getNNweights(    ice_msgs::getNNweights::Request& req,
			              ice_msgs::getNNweights::Response& resp );

	ros::ServiceServer updateInnerNNweights_srv_;
	ros::ServiceServer updateNNweights_srv_;
	ros::ServiceServer setNNweights_srv_;
	ros::ServiceServer getNNweights_srv_;

	ros::ServiceServer publish_srv_;
	ros::ServiceServer capture_srv_;
	ros::ServiceServer setRefTraj_srv_;
	ros::ServiceServer toggleFixedWeights_srv_;

	ros::ServiceServer status_srv_;
	bool statusCB( ice_msgs::setBool::Request & req,
				   ice_msgs::setBool::Response& resp );

	ros::ServiceServer tactileCalibration_srv_;

	bool tactileCalibrationCB(	ice_msgs::tactileCalibration::Request & req,
						    	ice_msgs::tactileCalibration::Response& resp );

	ros::ServiceServer tactileFilterWeights_srv_;

	bool tactileFilterWeightsCB(	ice_msgs::tactileFilterWeights::Request & req,
						    		ice_msgs::tactileFilterWeights::Response& resp );

	int tactileSensorSelected_;
	int numTactileSensors_;

	ros::ServiceServer runExperimentA_srv_;
	ros::ServiceServer runExperimentB_srv_;
	ros::ServiceServer runExperimentC_srv_;
	ros::ServiceServer runExperimentD_srv_;

	bool runExperimentA(	ice_msgs::setValue::Request & req,
						    ice_msgs::setValue::Response& resp );
	bool runExperimentB(	ice_msgs::setValue::Request & req,
						    ice_msgs::setValue::Response& resp );
	bool runExperimentC(	ice_msgs::setValue::Request & req,
						    ice_msgs::setValue::Response& resp );
	bool runExperimentD(	ice_msgs::setTrajectory::Request & req,
						    ice_msgs::setTrajectory::Response& resp );

	ice_msgs::setTrajectory traj_msgs_;

	Experiment experiment_;

	void bufferData();
	bool recordData;
	//  void setDataPoint(dataPoint::Datum* datum, double & dt);
	//  dataPoint::controllerFullData controllerData;

	bool accelerometerOn;

	bool executeCircleTraj;
	int numCircleTraj;

	bool initParam();
	bool initRobot();
	bool initTrajectories();
	bool initInnerLoop();
	bool initOuterLoop();
	bool initSensors();
	bool initNN();
	bool initNullspace();

	void updateOuterLoop();
	void updateNonRealtime();

	void readForceValuesCB(const geometry_msgs::WrenchStampedConstPtr& wrench_msg);
	void readTactileDataCB(const ice_msgs::tactileArrayDataConstPtr& msg);

	  realtime_tools::RealtimePublisher<StateMsg> pub_state_;
	  realtime_tools::RealtimePublisher<geometry_msgs::PoseStamped> pub_x_desi_;
	  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> pub_ft_;
	  realtime_tools::RealtimePublisher<geometry_msgs::WrenchStamped> pub_ft_transformed_;

	  int loop_count_;

	  boost::thread	m_Thread;
	  volatile bool runComputations;

	  CartVec cartvec_tmp;

public:
	PR2adaptNeuroControllerClass();
	~PR2adaptNeuroControllerClass();
	bool init(pr2_mechanism_model::RobotState *robot,
			ros::NodeHandle &n);
	void starting();
	void update();
	void stopping();

	void calcHumanIntentPos( Eigen::Vector3d & force, Eigen::VectorXd & pos, double delT, double m );

};
}
