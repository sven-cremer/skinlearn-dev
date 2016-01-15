#include "ice_robot_controllers/cartneuroController.h"
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS( pr2_controller_ns::PR2CartneuroControllerClass, pr2_controller_interface::Controller)

PR2CartneuroControllerClass::PR2CartneuroControllerClass()
: robot_state_(NULL)			// Initialize variables
{}

PR2CartneuroControllerClass::~PR2CartneuroControllerClass()
{
	sub_command_.shutdown();
}

/// Controller initialization in non-realtime
bool PR2CartneuroControllerClass::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
	ROS_INFO("Initializing Neuroadpative Controller...");

	// Store node handle
	nh_ = n;
	// Test if we got robot pointer
	assert(robot);
	// Store the robot handle for later use (to get time).
	robot_state_ = robot;

	// Verify that the version of the library that we linked against is
	// compatible with the version of the headers we compiled against.
	// GOOGLE_PROTOBUF_VERIFY_VERSION;

	if( !initParam() )
	{
		ROS_ERROR("initParam() failed!");
		return false;
	}
	if( !initRobot() )
	{
		ROS_ERROR("initRobot() failed!");
		return false;
	}
	if( !initSensors() )
	{
		ROS_ERROR("initSensors() failed!");
		return false;
	}
	if( !initTrajectories() )
	{
		ROS_ERROR("initTrajectories() failed!");
		return false;
	}
	if( !initInnerLoop() )
	{
		ROS_ERROR("initInnerLoop() failed!");
		return false;
	}
	if( !initOuterLoop() )
	{
		ROS_ERROR("initOuterLoop() failed!");
		return false;
	}
	if( !initNN() )
	{
		ROS_ERROR("initNN() failed!");
		return false;
	}

	// Subscribe to Flexiforce wrench commands
	if(useFlexiForce)
		sub_command_ = nh_.subscribe<geometry_msgs::Wrench>("command", 1, &PR2CartneuroControllerClass::readForceValuesCB, this);

	/////////////////////////
	// DATA COLLECTION
	save_srv_                = nh_.advertiseService("save"               , &PR2CartneuroControllerClass::save                 , this);
	publish_srv_             = nh_.advertiseService("publish"            , &PR2CartneuroControllerClass::publish              , this);
	capture_srv_             = nh_.advertiseService("capture"            , &PR2CartneuroControllerClass::capture              , this);
	setRefTraj_srv_          = nh_.advertiseService("setRefTraj"         , &PR2CartneuroControllerClass::setRefTraj           , this);
	toggleFixedWeights_srv_  = nh_.advertiseService("toggleFixedWeights" , &PR2CartneuroControllerClass::toggleFixedWeights   , this);

	pubFTData_               = nh_.advertise< geometry_msgs::WrenchStamped >( "FT_data"              , StoreLen);
	pubModelStates_          = nh_.advertise< sensor_msgs::JointState      >( "model_joint_states"   , StoreLen);
	pubRobotStates_          = nh_.advertise< sensor_msgs::JointState      >( "robot_joint_states"   , StoreLen);
	pubModelCartPos_         = nh_.advertise< geometry_msgs::PoseStamped   >( "model_cart_pos"       , StoreLen);
	pubRobotCartPos_         = nh_.advertise< geometry_msgs::PoseStamped   >( "robot_cart_pos"       , StoreLen);
	pubControllerParam_      = nh_.advertise< ice_msgs::controllerParam    >( "controller_params"    , StoreLen);
	pubControllerFullData_   = nh_.advertise< ice_msgs::controllerFullData >( "controllerFullData"   , StoreLen);

	storage_index_ = StoreLen;
	// DATA COLLECTION END
	/////////////////////////



	  StateMsg state_template;
	  state_template.header.frame_id = root_name;
	  state_template.x.header.frame_id = root_name;
	  state_template.x_desi.header.frame_id = root_name;
	  state_template.x_desi_filtered.header.frame_id = root_name;
	  state_template.q.resize(Joints);
	  state_template.tau_c.resize(Joints);
	  state_template.tau_posture.resize(Joints);
	  state_template.J.layout.dim.resize(2);
	  state_template.J.data.resize(6*Joints);
	  state_template.N.layout.dim.resize(2);
	  state_template.N.data.resize(Joints*Joints);
	  // NN weights
	  state_template.W.layout.dim.resize(2);
	  state_template.W.data.resize(num_Outputs*num_Hidden);
	  state_template.V.layout.dim.resize(2);
	  state_template.V.data.resize(num_Hidden*(num_Inputs+1));
	  pub_state_.init(nh_, "state", 10);
	  pub_state_.lock();
	  pub_state_.msg_ = state_template;
	  pub_state_.unlock();

	  pub_x_desi_.init(nh_, "state/x_desi", 10);
	  pub_x_desi_.lock();
	  pub_x_desi_.msg_.header.frame_id = root_name;
	  pub_x_desi_.unlock();

	  pub_ft_.init(nh_, "ft/l_gripper",10);
	  pub_ft_.lock();
	  pub_ft_.msg_.header.frame_id = ft_frame_id;
	  pub_ft_.unlock();

	  pub_ft_transformed_.init(nh_, "ft/l_gripper_transformed",10);
	  pub_ft_transformed_.lock();
	  pub_ft_transformed_.msg_.header.frame_id = root_name;
	  pub_ft_transformed_.unlock();


	ROS_INFO("Neuroadpative Controller is initialized!");
	return true;
}


/// Controller startup in realtime
void PR2CartneuroControllerClass::starting()
{

	// Get the current joint values to compute the initial tip location.
	chain_.getPositions(q0_);
	kin_->fk(q0_, x0_);


	/////////////////////////
	// System Model

	// System Model END
	/////////////////////////

	// Initialize the phase of the circle as zero.
	circle_phase_ = 0.0;
	startCircleTraj = true;

	// Also reset the time-of-last-servo-cycle.
	last_time_     = robot_state_->getTime() ;
	start_time_    = robot_state_->getTime() ;
	outer_elapsed_ = robot_state_->getTime() ;

	// Move arm into position
	if( !useCurrentCartPose ) // TODO this was moved from update() ... is this correct? The controller seems to oscillate now.
	{
		// Start from specified
		Eigen::Vector3d p_init(cartIniX,cartIniY,cartIniZ);
		Eigen::Quaterniond q_init = euler2Quaternion( cartDesRoll, cartDesPitch, cartDesYaw );
		x0_ = Eigen::Translation3d(p_init) * q_init;
	}
	x_d_ = x0_;

	ROS_INFO("Starting pose: [%f,%f,%f]",x_d_.translation().x(),x_d_.translation().y(),x_d_.translation().z());


	biasMeasured = false;
	geometry_msgs::Wrench tmp;	// Initialize with zero
	l_ftBias.wrench = tmp;
	r_ftBias.wrench = tmp;


	qdot_filtered_.setZero();
	joint_vel_filter_ = 1.0;

	loop_count_ = 0;
}


/// Controller update loop in realtime
void PR2CartneuroControllerClass::update()
{

	double dt;                    // Servo loop time step
	++loop_count_;

	// Calculate the dt between servo cycles.
	dt = (robot_state_->getTime() - last_time_).toSec();
	last_time_ = robot_state_->getTime();

	// Get the current joint positions and velocities.
	chain_.getPositions(q_);
	chain_.getVelocities(qdot_);

	// Get the pose of the F/T sensor
	if(forceTorqueOn)
		kin_ft_->fk(q_,x_ft_);

	// Compute the forward kinematics and Jacobian (at this location).
	kin_->fk(q_, x_);
	kin_->jac(q_, J_);
	kin_acc_->jac(q_, J_acc_);

	chain_.getVelocities(qdot_raw_);
	for (int i = 0; i < Joints; ++i)
		qdot_filtered_[i] += joint_vel_filter_ * (qdot_raw_[i] - qdot_filtered_[i]);	// Does nothing when joint_vel_filter_=1
	qdot_ = qdot_filtered_;
	xdot_ = J_ * qdot_;

	xdot_ = J_ * qdot_raw_;		// FIXME remove

	// Get accelerometer forward kinematics
	kin_acc_->fk(q_, x_gripper_acc_);

	// Estimate force at gripper tip
//	chain_.getEfforts(tau_measured_);
//	for (unsigned int i = 0 ; i < 6 ; i++)
//	{
//		force_measured_(i) = 0;
//		for (unsigned int j = 0 ; j < Joints ; j++)
//			force_measured_(i) += J_(i,j) * tau_measured_(j);
//	}

	///////////////////////////////
	// Human force input
	// Force error

	/***************** SENSOR UPDATE *****************/

	if(accelerometerOn)
	{
		l_accelerationObserver->spin();
		r_accelerationObserver->spin();

		// retrieve right accelerometer data
		std::vector<geometry_msgs::Vector3> rightGripperAcc = r_accelerometer_handle_->state_.samples_;

		r_acc_data( 0 ) = r_accelerationObserver->aX_lp ; // threeAccs[threeAccs.size()-1].x ;
		r_acc_data( 1 ) = r_accelerationObserver->aY_lp ; // threeAccs[threeAccs.size()-1].y ;
		r_acc_data( 2 ) = r_accelerationObserver->aZ_lp ; // threeAccs[threeAccs.size()-1].z ;

		//    rightGripperAcc.clear(); // Do we need this?
	}

	if(useFlexiForce)
	{
		// Read flexi force serial data
		//  tacSerial->getDataArrayFromSerialPort( flexiForce );
		flexiForce(0) = flexiforce_wrench_desi_.force(0) ;			// TODO change message type
		flexiForce(1) = flexiforce_wrench_desi_.force(1) ;			// note: this variable is being updated by the readForceValuesCB
		flexiForce(2) = flexiforce_wrench_desi_.force(2) ;
		flexiForce(3) = flexiforce_wrench_desi_.torque(0);

		//             0
		//
		//     1   >   ^    >   3 y
		//
		//             2
		//             x

		// X axis
		if( flexiForce(0) > flexiForce(2) ){
			FLEX_force(0) = -flexiForce(0);
		}else{
			FLEX_force(0) =  flexiForce(2);
		}
		// Y axis
		if( flexiForce(1) > flexiForce(3) ){
			FLEX_force(1) =  flexiForce(1);
		}else{
			FLEX_force(1) = -flexiForce(3);
		}
		// Z axis
		FLEX_force(2) = 0;

		transformed_force = FLEX_force;
	}

	if( forceTorqueOn )		// TODO only use left or right arm
	{
		std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
		l_ft_samples    = l_ftData_vector.size() - 1;

		std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
		r_ft_samples    = r_ftData_vector.size() - 1;

		// Measure bias after the arm is in position
		if( !biasMeasured &&  loop_count_> 3000)
		{
			// set FT sensor bias due to gravity				// FIXME this will change as the gripper moves
			l_ftBias.wrench = l_ftData_vector[l_ft_samples];	// FIXME use a moving average
			r_ftBias.wrench = r_ftData_vector[r_ft_samples];

			biasMeasured = true;
		}

		//      l_ftData.wrench = l_ftData_vector[l_ft_samples];
		l_ftData.wrench.force.x  = l_ftData_vector[l_ft_samples].force.x  - l_ftBias.wrench.force.x ;
		l_ftData.wrench.force.y  = l_ftData_vector[l_ft_samples].force.y  - l_ftBias.wrench.force.y ;
		l_ftData.wrench.force.z  = l_ftData_vector[l_ft_samples].force.z  - l_ftBias.wrench.force.z ;
		l_ftData.wrench.torque.x = l_ftData_vector[l_ft_samples].torque.x - l_ftBias.wrench.torque.x;
		l_ftData.wrench.torque.y = l_ftData_vector[l_ft_samples].torque.y - l_ftBias.wrench.torque.y;
		l_ftData.wrench.torque.z = l_ftData_vector[l_ft_samples].torque.z - l_ftBias.wrench.torque.z;

		//      r_ftData.wrench = r_ftData_vector[r_ft_samples];
		r_ftData.wrench.force.x  =   ( r_ftData_vector[r_ft_samples].force.x  - r_ftBias.wrench.force.x  ) ;
		r_ftData.wrench.force.y  =   ( r_ftData_vector[r_ft_samples].force.y  - r_ftBias.wrench.force.y  ) ;
		r_ftData.wrench.force.z  =   ( r_ftData_vector[r_ft_samples].force.z  - r_ftBias.wrench.force.z  ) ;
		r_ftData.wrench.torque.x =   ( r_ftData_vector[r_ft_samples].torque.x - r_ftBias.wrench.torque.x ) ;
		r_ftData.wrench.torque.y =   ( r_ftData_vector[r_ft_samples].torque.y - r_ftBias.wrench.torque.y ) ;
		r_ftData.wrench.torque.z =   ( r_ftData_vector[r_ft_samples].torque.z - r_ftBias.wrench.torque.z ) ;

		//  Eigen::Vector3d forceFT( r_ftData.wrench.force.x, r_ftData.wrench.force.y, r_ftData.wrench.force.z );

//		rosrun tf tf_echo l_force_torque_link l_gripper_tool_frame
//		- Translation: [-0.000, 0.000, -0.180]
//		- Rotation: in Quaternion [0.406, 0.579, -0.406, 0.579]
//		            in RPY [3.142, 1.571, 1.919]
//		CartVec tf_tool;
//		tf_tool(0) = 0;
//		tf_tool(1) = 0;
//		tf_tool(2) = 0;
//		tf_tool(3) = 3.142;
//		tf_tool(4) = 1.571;
//		tf_tool(5) = 1.919;
		tf::wrenchMsgToEigen(l_ftData.wrench, wrench_);

		//wrench_ = affine2CartVec(CartVec2Affine(tf_tool)*CartVec2Affine(wrench_));

//		Eigen::Vector3d forceFT( l_ftData.wrench.force.x, l_ftData.wrench.force.y, l_ftData.wrench.force.z );
//		Eigen::Vector3d tauFT( l_ftData.wrench.torque.x, l_ftData.wrench.torque.y, l_ftData.wrench.torque.z );

//		forceFT << l_ftData.wrench.force.x, l_ftData.wrench.force.y, l_ftData.wrench.force.z;
//		tauFT << l_ftData.wrench.torque.x, l_ftData.wrench.torque.y, l_ftData.wrench.torque.z;

		forceFT(0) = l_ftData.wrench.force.x;
		forceFT(1) = l_ftData.wrench.force.y;
		forceFT(2) = l_ftData.wrench.force.z;

		tauFT(0) = l_ftData.wrench.torque.x;
		tauFT(1) = l_ftData.wrench.torque.y;
		tauFT(2) = l_ftData.wrench.torque.z;

		// **************************************
		// Force transformation
		// FIXME this code produces the correct results but crashes the RT loop
//		double px = x_ft_.translation().x();
//		double py = x_ft_.translation().y();
//		double pz = x_ft_.translation().z();
//		W_mat_ << 0,-pz,py,
//			      pz,0,-px,
//			      -py,px,0;

//		W_mat_ << 0, -x_ft_.translation().z(), x_ft_.translation().y(),
//				  x_ft_.translation().z(), 0, -x_ft_.translation().x(),
//			      -x_ft_.translation().y(), x_ft_.translation().x(), 0;

		W_mat_(0,0) = 0;
		W_mat_(0,1) = -x_ft_.translation().z();
		W_mat_(0,2) = x_ft_.translation().y();

		W_mat_(1,0) = x_ft_.translation().z();
		W_mat_(1,1) = 0;
		W_mat_(1,2) = -x_ft_.translation().x();

		W_mat_(2,0) = -x_ft_.translation().y();
		W_mat_(2,1) = x_ft_.translation().x();
		W_mat_(2,2) = 0;

		tauTorso = W_mat_*x_ft_.linear()*forceFT + x_ft_.linear()*tauFT;
		forceTorso = x_ft_.linear()*forceFT;

		wrench_transformed_(0) = forceTorso(0);
		wrench_transformed_(1) = forceTorso(1);
		wrench_transformed_(2) = forceTorso(2);
		wrench_transformed_(3) = tauTorso(0);
		wrench_transformed_(4) = tauTorso(1);
		wrench_transformed_(5) = tauTorso(2);

		// **************************************

		//                               w       x       y      z
//		Eigen::Quaterniond ft_to_acc(0.579, -0.406, -0.579, 0.406);					// FIXME is this correct? right vs left?
//		FT_transformed_force = ft_to_acc._transformVector( forceFT );				// TODO add wrench
//		FT_transformed_force(1) = - FT_transformed_force(1);						// This should be in torso_lift_link (root_name)
//		transformed_force = FT_transformed_force;

		transformed_force = forceTorso;
		//transformed_force = Eigen::Vector3d::Zero();
	}

	// Force threshold (makes force zero bellow threshold)
	if( ( transformed_force(0) < forceCutOffX ) && ( transformed_force(0) > -forceCutOffX ) ){ transformed_force(0) = 0; }
	if( ( transformed_force(1) < forceCutOffY ) && ( transformed_force(1) > -forceCutOffY ) ){ transformed_force(1) = 0; }
	if( ( transformed_force(2) < forceCutOffZ ) && ( transformed_force(2) > -forceCutOffZ ) ){ transformed_force(2) = 0; }

	// Human force input END
	///////////////////////////////

	/***************** UPDATE *****************/

	if(executeCircleTraj)
	{
		// Follow a circle of 10cm at 0.4 rad/sec.
		circle_phase_ += 0.4 * dt;

		Eigen::Matrix<double, 3, 1> circle(0,0,0);
		circle.y() = 0.10 * sin(circle_phase_);
		circle.z() = 0.10 * (1 - cos(circle_phase_));

		x_d_ = x0_;
		x_d_.translation() += circle;
	}
	//double circleAmpl = (circleUlim - circleLlim)/2 ;


	// Calculate a Cartesian restoring force.
	computePoseError(x_, x_d_, xerr_);			// TODO: Use xd_filtered_ instead


	// Current joint positions and velocities
	q = q_;
	qd = qdot_;

	X = affine2CartVec(x_);
	Xd = xdot_;				// FIXME make sure they are of same type

	X_m = affine2CartVec(x_d_);
	//	CartVec xyzrpy = affine2CartVec(xd_T);
	//  X_m(0) = xd_.p(0);  Xd_m(0) = 0;  Xdd_m(0) = 0;		// FIXME should X_m be updated?
	//  X_m(1) = xd_.p(1);  Xd_m(1) = 0;  Xdd_m(1) = 0;
	//  X_m(2) = xd_.p(2);  Xd_m(2) = 0;  Xdd_m(2) = 0;
	//	X_m(3) = xyzrpy(3);  Xd_m(3) = 0;  Xdd_m(3) = 0;
	//	X_m(4) = xyzrpy(4);  Xd_m(4) = 0;  Xdd_m(4) = 0;
	//	X_m(5) = xyzrpy(5);  Xd_m(5) = 0;  Xdd_m(5) = 0;
	//X_m   = Eigen::VectorXd::Zero(6);

	Xd_m  = Eigen::VectorXd::Zero(6);
	Xdd_m = Eigen::VectorXd::Zero(6);

	/***************** SYSTEM MODEL *****************/

	// USed to auto set cart pose
	/*    if( (int) ceil( (robot_state_->getTime() - start_time_).toSec() ) % 3 == 0 )
    {
      task_ref(0) = cartDesX ;
      task_ref(1) = cartDesY ;
      task_ref(2) = cartDesZ ;
    }

    if( (int) ceil( (robot_state_->getTime() - start_time_).toSec() ) % 6 == 0 )
    {
      task_ref(0) = cartIniX ;
      task_ref(1) = cartIniY ;
      task_ref(2) = cartIniZ ;
    }*/

//	if( !forceTorqueOn )
//	{
		/////////////////////////
		// Simulated human model

		/*
      // Open loop version
      ode_init_x[2] = task_ref(1); // q_r
      ode_init_x[3] = 0          ; // qd_r

      // Closed loop version
      // ode_init_x[2] = ( task_ref(1) - X_m(1) ); // q_r
      // ode_init_x[3] = 0                       ; // - Xd (1)                ; // qd_r

  //    boost::numeric::odeint::integrate( human_model , ode_init_x , 0.0 , delT , delT );
  //    transformed_force(1) = ode_init_x[0];

//      double T  = 0.18;
//      double Kp = 779 ;
//      double Kd = 288 ;
//       Bakur's values ?
      double T  = 1;
      double Kp = 1 ;
      double Kd = 0 ;

      double a_h = 10000;
      double b_h = a_h;

      // Reduced human model
      transformed_force (1) = transformed_force (1) + ode_init_x[1] * delT;

      //               ( Kp q_r           + Kd qd_r          - hf                    ) / T
      // ode_init_x[1] = ( Kp*ode_init_x[2] + Kd*ode_init_x[3] - transformed_force (1) ) / T ;
      ode_init_x[1] = b_h*task_ref(1) - a_h*transformed_force (1);


      ROS_ERROR_STREAM("USING Simulated human model");
		 */

		// END Simulated human model
		/////////////////////////
//	}

	/***************** REFERENCE TRAJECTORY *****************/

	if(forceTorqueOn)
	{
		t_r(0) = transformed_force(0) ;			// FIXME make sure orientation is correct
		t_r(1) = transformed_force(1) ;
//	    t_r(2) = transformed_force(2) ;
//	    t_r(3) = r_ftData.wrench.torque.x;		// FIXME transform wrenches
//	    t_r(4) = r_ftData.wrench.torque.y;
//	    t_r(5) = r_ftData.wrench.torque.z;
	}
//	else
//	{
//		t_r = force_measured_;					// Computed from joint efforts
//	}

	t_r = Eigen::VectorXd::Zero(6);				// FIXME inner loop only works if t_r = 0


//	CartVec tmp;
//	tmp(0) = l_ftData.wrench.force.x  ;
//	tmp(1) = l_ftData.wrench.force.y  ;
//	tmp(2) = l_ftData.wrench.force.z  ;
//	tmp(3) = l_ftData.wrench.torque.x ;
//	tmp(4) = l_ftData.wrench.torque.y ;
//	tmp(5) = l_ftData.wrench.torque.z ;

//	t_r = J_acc_.transpose() * tmp;

	/***************** OUTER LOOP *****************/

	//updateOuterLoop();

	/***************** INNER LOOP *****************/

	// Neural Network
	nnController.UpdateCart( X     ,
                             Xd    ,
                             X_m   ,
                             Xd_m  ,
                             Xdd_m ,
                             q     ,
                             qd    ,
                             t_r   ,			// Human force
                             force_c  );		// Output


	// PD controller
	/*
	CartVec kp, kd;
	kp << 100.0,100.0,100.0,100.0,100.0,100.0;
	kd << 1.0,1.0,1.0,1.0,1.0,1.0;
	// F    = -(       Kp * (x-x_dis)   +     Kd * (xdot - 0)    )
	force_c = -(kp.asDiagonal() * xerr_ + kd.asDiagonal() * xdot_);			// TODO choose NN/PD with a param
	*/

	JacobianTrans = J_.transpose();

	tau = JacobianTrans*force_c;

	/***************** NULLSPACE *****************/

	// Code taken from JT Cartesian controller

	// Computes pseudo-inverse of J
	Eigen::Matrix<double,6,6> I6; I6.setIdentity();
	Eigen::Matrix<double,6,6> JJt_damped = J_ * JacobianTrans + jacobian_inverse_damping * I6;
	Eigen::Matrix<double,6,6> JJt_inv_damped = JJt_damped.inverse();
	Eigen::Matrix<double,Joints,6> J_pinv = JacobianTrans * JJt_inv_damped;

	// Computes the nullspace of J
	Eigen::Matrix<double,Joints,Joints> I;
	I.setIdentity();
	nullSpace = I - J_pinv * J_;

	  // Computes the desired joint torques for achieving the posture
	nullspaceTorque.setZero();
	if (useNullspacePose)
	{
		if(false)
		{
			// This doesn't seem to work
			JointVec posture_err = q_posture_ - q_;
			for (int j = 0; j < Joints; ++j)
			{
				if (chain_.getJoint(j)->joint_->type == urdf::Joint::CONTINUOUS)
					posture_err[j] = angles::normalize_angle(posture_err[j]);		// -PI to +PI
			}

			for (int j = 0; j < Joints; ++j) {
				if (fabs(q_posture_[j] - 9999) < 1e-5)
					posture_err[j] = 0.0;
			}

			JointVec qdd_posture = k_posture* posture_err;
			nullspaceTorque = joint_dd_ff_.array() * (nullSpace * qdd_posture).array();
		}
		else
		{

			//		qnom(0) = -0.5   ;
			//		qnom(1) =  0 ;
			//		qnom(2) = -1.50   ;
			//		qnom(3) = -1.7   ;
			//		qnom(4) =  1.50   ;
			//		qnom(5) =  0 ;
			//		qnom(6) =  0 ;

			JointVec q_jointLimit ;
			for (int j = 0; j < Joints; ++j)
			{
				///////////////////
				// Liegeois
				// This is the Liegeois cost function from 1977
				q_jointLimit(j) = - (q(j) - qnom(j) )/( q.rows() * ( q_upper(j) - q_lower(j))) ;
				// END Liegeois
				///////////////////
			}
			nullspaceTorque = nullSpace*50*( q_jointLimit - 0.0*qd );
		}

		tau = tau + nullspaceTorque;
	}



   /***************** TORQUE *****************/

	// Torque Saturation							(if a torque command is larger than the saturation value, then scale all torque values such that torque_i=saturation_i)
	double sat_scaling = 1.0;
	for (int i = 0; i < num_Joints; ++i) {
		if (saturation_[i] > 0.0)
			sat_scaling = std::min(sat_scaling, fabs(saturation_[i] / tau[i]));
	}
	Eigen::VectorXd tau_sat = sat_scaling * tau;

	tau_c_ = JointEigen2Kdl( tau_sat );

	// Send torque command
	chain_.setEfforts( tau_c_ );


   /***************** DATA COLLECTION *****************/

	//bufferData( dt );


   /***************** DATA PUBLISHING *****************/


	  if (loop_count_ % 10 == 0 && publishRTtopics)
	  {
	    if (pub_x_desi_.trylock()) {
	      pub_x_desi_.msg_.header.stamp = last_time_;
	      //tf::poseEigenToMsg(CartVec2Affine(X_m), pub_x_desi_.msg_.pose);
	      tf::poseEigenToMsg(x_ft_, pub_x_desi_.msg_.pose);
	      pub_x_desi_.unlockAndPublish();
	    }

	    /*
	    if (pub_state_.trylock()) {
	      // Headers
	      pub_state_.msg_.header.stamp = last_time_;
	      pub_state_.msg_.x.header.stamp = last_time_;
	      pub_state_.msg_.x_desi_filtered.header.stamp = last_time_;
	      pub_state_.msg_.x_desi.header.stamp = last_time_;
	      // Pose
	      tf::poseEigenToMsg(x_, pub_state_.msg_.x.pose);
	      tf::poseEigenToMsg(CartVec2Affine(X_m), pub_state_.msg_.x_desi.pose);		// X_m, xd_T
//	      tf::poseEigenToMsg(x_desi_filtered_, pub_state_.msg_.x_desi_filtered.pose);
	      // Error
	      tf::twistEigenToMsg(xerr_, pub_state_.msg_.x_err);
	      // Twist
	      tf::twistEigenToMsg(xdot_, pub_state_.msg_.xd);
//	      tf::twistEigenToMsg(Xd_m, pub_state_.msg_.xd_desi);
	      // Force
	      tf::wrenchEigenToMsg(t_r, pub_state_.msg_.force_measured);		// force_measured, t_r
	      tf::wrenchEigenToMsg(force_c, pub_state_.msg_.force_c);

	      tf::matrixEigenToMsg(J_, pub_state_.msg_.J);
	      tf::matrixEigenToMsg(nullSpace, pub_state_.msg_.N);

	      tf::matrixEigenToMsg(nnController.getInnerWeights(), pub_state_.msg_.V);
	      tf::matrixEigenToMsg(nnController.getOuterWeights(), pub_state_.msg_.W);

	      for (int j = 0; j < Joints; ++j) {
	        pub_state_.msg_.tau_posture[j] = nullspaceTorque[j];
	        pub_state_.msg_.tau_c[j] = tau[j];
	        pub_state_.msg_.q[j] = q_[j];
	      }
	      pub_state_.msg_.W_norm = nnController.getInnerWeightsNorm();
	      pub_state_.msg_.V_norm = nnController.getOuterWeightsNorm();

	      pub_state_.unlockAndPublish();
	    }
	    */

	    if (pub_ft_.trylock()) {
	    	pub_ft_.msg_.header.stamp = last_time_;
	    	tf::wrenchEigenToMsg(wrench_, pub_ft_.msg_.wrench);
	    	//pub_ft_.msg_.wrench = l_ftData.wrench;
	    	pub_ft_.unlockAndPublish();
	    }

	    if (pub_ft_transformed_.trylock()) {
	    	pub_ft_transformed_.msg_.header.stamp = last_time_;
	    	tf::wrenchEigenToMsg(wrench_transformed_, pub_ft_transformed_.msg_.wrench);
	    	pub_ft_transformed_.unlockAndPublish();
	    }

	  }


}


void PR2CartneuroControllerClass::updateOuterLoop()
{
	// OUTER Loop Update

	// Human Intent Estimation
	if( !externalRefTraj )
	{
		if( ( robot_state_->getTime() - intent_elapsed_ ).toSec() >= intentLoopTime )
		{
			calcHumanIntentPos( transformed_force, task_ref, intentEst_delT, intentEst_M );

			// Transform human intent to torso lift link
			CartVec xyz = affine2CartVec(x_gripper_acc_);
			task_ref.x() = xyz(1) + task_ref.x() ;
			task_ref.y() = xyz(2) + task_ref.y() ;
			task_ref.z() = xyz(3) + task_ref.z() ;

			intent_elapsed_ = robot_state_->getTime() ;
		}
	}


	if( ( robot_state_->getTime() - outer_elapsed_ ).toSec() >= outerLoopTime )
	{
		// RLS ARMA
		if( useARMAmodel )
		{
			if( useFlexiForce )
			{
				// Set ARMA parameters
				// X axis
				if( flexiForce(0) > flexiForce(2) ){

					outerLoopRLSmodelX.setWeights( outerLoopWk_flexi_1 ) ;
					if( useFixedWeights )
						outerLoopRLSmodelX.setFixedWeights( outerLoopWk_flexi_1 );
					else
						outerLoopRLSmodelX.setUpdatedWeights();

				}else{
					outerLoopRLSmodelX.setWeights( outerLoopWk_flexi_3 ) ;
					if( useFixedWeights )
						outerLoopRLSmodelX.setFixedWeights( outerLoopWk_flexi_3 );
					else
						outerLoopRLSmodelX.setUpdatedWeights();
				}

				// Y axis
				if( flexiForce(1) > flexiForce(3) ){
					outerLoopRLSmodelY.setWeights( outerLoopWk_flexi_2 ) ;
					if( useFixedWeights )
						outerLoopRLSmodelY.setFixedWeights( outerLoopWk_flexi_2 );
					else
						outerLoopRLSmodelY.setUpdatedWeights();
				}else{
					outerLoopRLSmodelY.setWeights( outerLoopWk_flexi_4 ) ;
					if( useFixedWeights )
						outerLoopRLSmodelY.setFixedWeights( outerLoopWk_flexi_4 );
					else
						outerLoopRLSmodelY.setUpdatedWeights();
				}
			}

			// X axis
			outerLoopRLSmodelX.updateARMA( Xd_m                   (0) ,
                                           Xd                     (0) ,
                                           X_m                    (0) ,
                                           X                      (0) ,
                                           Xdd_m                  (0) ,
                                           transformed_force      (0) ,
                                           task_ref               (0) ,
                                           task_refModel_output   (0)  );

			// Y axis
			outerLoopRLSmodelY.updateARMA( Xd_m                   (1) ,
                                           Xd                     (1) ,
                                           X_m                    (1) ,
                                           X                      (1) ,
                                           Xdd_m                  (1) ,
                                           transformed_force      (1) ,
                                           task_ref               (1) ,
                                           task_refModel_output   (1)  );

			// ROS_ERROR_STREAM("USING RLS ARMA");

			if( useFlexiForce )
			{
				// X axis
				if( flexiForce(0) > flexiForce(2) )
				{
					outerLoopRLSmodelX.getWeights( outerLoopWk_flexi_1 ) ;
				}else{
					outerLoopRLSmodelX.getWeights( outerLoopWk_flexi_3 ) ;
				}

				// Y axis
				if( flexiForce(1) > flexiForce(3) )
				{
					outerLoopRLSmodelY.getWeights( outerLoopWk_flexi_2 ) ;
				}else{
					outerLoopRLSmodelY.getWeights( outerLoopWk_flexi_4 ) ;
				}

			}else
			{
				outerLoopRLSmodelX.getWeights( outerLoopWk ) ;
				outerLoopRLSmodelY.getWeights( outerLoopWk ) ;
			}

		}

		// CT RLS ARMA
		if( useCTARMAmodel )
		{
			//      outerLoopCTRLSmodelX.updateARMA( Xd_m              (0) ,
			//                                       Xd                (0) ,
			//                                       X_m               (0) ,
			//                                       X                 (0) ,
			//                                       Xdd_m             (0) ,
			//                                       transformed_force (0) ,
			//                                       task_ref          (0) ,
			//                                       task_refModel     (0)  );

			// Y axis
			outerLoopCTRLSmodelY.updateARMA( Xd_m                   (1) ,
                                             Xd                     (1) ,
                                             X_m                    (1) ,
                                             X                      (1) ,
                                             Xdd_m                  (1) ,
                                             transformed_force      (1) ,
                                             task_ref               (1) ,
                                             task_refModel_output   (1)  );

			//      ROS_ERROR_STREAM("USING CT RLS ARMA");

			outerLoopCTRLSmodelY.getWeights( outerLoopWk ) ;
			//		outerLoopCTRLSmodelY.setWeights( outerLoopWk ) ;
		}

		// RLS FIR
		if( useFIRmodel )
		{
			//      outerLoopRLSmodelX.updateFIR( Xd_m              (0) ,
			//                                    Xd                (0) ,
			//                                    X_m               (0) ,
			//                                    X                 (0) ,
			//                                    Xdd_m             (0) ,
			//                                    transformed_force (0) ,
			//                                    task_ref          (0) ,
			//                                    task_refModel     (0)  );

			// Y axis
			outerLoopRLSmodelY.updateFIR( Xd_m                   (1) ,
                                          Xd                     (1) ,
                                          X_m                    (1) ,
                                          X                      (1) ,
                                          Xdd_m                  (1) ,
                                          transformed_force      (1) ,
                                          task_ref               (1) ,
                                          task_refModel_output   (1)  );
			//      ROS_ERROR_STREAM("USING RLS FIR");

			outerLoopRLSmodelY.getWeights( outerLoopWk ) ;
			//		outerLoopRLSmodelY.setWeights( outerLoopWk ) ;
		}
		/*
			// MRAC
			if( useMRACmodel )
			{
		//      outerLoopMRACmodelX.update( Xd_m              (0) ,
		//                                  Xd                (0) ,
		//                                  X_m               (0) ,
		//                                  X                 (0) ,
		//                                  Xdd_m             (0) ,
		//                                  transformed_force (0) ,
		//                                  task_ref          (0) ,
		//                                  task_refModel     (0)  );

			  // Y axis
			  outerLoopMRACmodelY.update( Xd_m              (1) ,
										  Xd                (1) ,
										  X_m               (1) ,
										  X                 (1) ,
										  Xdd_m             (1) ,
										  transformed_force (1) ,
										  task_ref          (1) ,
										  task_refModel     (1)  );

		//      ROS_ERROR_STREAM("USING MRAC");
			}
		 */
		// MSD
		if( useMSDmodel )
		{
			// Cartesian space MSD model
			outerLoopMSDmodelX.update( Xd_m             (0) ,
                                       Xd               (0) ,
                                       X_m              (0) ,
                                       X                (0) ,
                                       Xdd_m            (0) ,
                                       transformed_force(0)  );

			outerLoopMSDmodelY.update( Xd_m             (1) ,
                                       Xd               (1) ,
                                       X_m              (1) ,
                                       X                (1) ,
                                       Xdd_m            (1) ,
                                       transformed_force(1)  );
			//      ROS_ERROR_STREAM("USING MSD");
		}

		// IRL
		if( useIRLmodel )
		{
			//    // Cartesian space IRL model
			//    outerLoopIRLmodelX.updateIRL( Xd_m              (0) ,
			//   	                            Xd                (0) ,
			//   	                            X_m               (0) ,
			//   	                            X                 (0) ,
			//   	                            Xdd_m             (0) ,
			//   	                            transformed_force (0) ,
			//   	                            task_ref          (0) ,
			//   	                            task_refModel     (0)  );

			outerLoopIRLmodelY.updateIRL( Xd_m                  (1) ,
                                          Xd                    (1) ,
                                          X_m                   (1) ,
                                          X                     (1) ,
                                          Xdd_m                 (1) ,
                                          transformed_force     (1) ,
                                          task_ref              (1) ,
                                          task_refModel_output  (1)  );


			// IRL
			//			outerLoopIRLmodelX.getMsd( m_M,
			//			                           m_S,
			//			                           m_D );
			outerLoopIRLmodelY.getMsd( m_M,
                                       m_S,
                                       m_D );

			//      ROS_ERROR_STREAM("USING MSD");
		}

		// Direct Model
		if( useDirectmodel )
		{
			// q_d
			X_m(1)= task_refModel_output(1)      ;
			Xd_m  = (X_m - p_X_m)/outerLoopTime  ;
			Xdd_m = (Xd_m - p_Xd_m)/outerLoopTime;
		}

		// MRAC
		if( useMRACmodel )
		{
			//      outerLoopMRACmodelX.update( Xd_m              (0) ,
			//                                  Xd                (0) ,
			//                                  X_m               (0) ,
			//                                  X                 (0) ,
			//                                  Xdd_m             (0) ,
			//                                  transformed_force (0) ,
			//                                  task_ref          (0) ,
			//                                  task_refModel     (0)  );

			// Y axis
			outerLoopMRACmodelY.update( Xd_m                 (1) ,
                                        Xd                   (1) ,
                                        X_m                  (1) ,
                                        X                    (1) ,
                                        Xdd_m                (1) ,
                                        transformed_force    (1) ,
                                        task_ref             (1) ,
                                        task_refModel_output (1)  );

			//      ROS_ERROR_STREAM("USING MRAC");
		}

		p_X_m    = X_m   ;
		p_Xd_m   = Xd_m  ;
		p_Xdd_m  = Xdd_m ;

		outer_elapsed_ = robot_state_->getTime() ;

	}

	// System Model END
	/////////////////////////
}

void PR2CartneuroControllerClass::bufferData( double & dt )
{
	int index = storage_index_;
	if ((index >= 0) && (index < StoreLen))
	{
		//                tf::PoseKDLToMsg(x_m_, modelCartPos_);
		//                tf::PoseKDLToMsg(x_  , robotCartPos_);

		msgControllerFullData[index].dt                = dt                          ;

		// Force Data
		msgControllerFullData[index].force_x           = transformed_force(0)        ; // r_ftData.wrench.force.x     ;
		msgControllerFullData[index].force_y           = transformed_force(1)        ; // r_ftData.wrench.force.y     ;
		msgControllerFullData[index].force_z           = transformed_force(2)        ; // r_ftData.wrench.force.z     ;
		msgControllerFullData[index].torque_x          = 0                           ; // r_ftData.wrench.torque.x    ;
		msgControllerFullData[index].torque_y          = 0                           ; // r_ftData.wrench.torque.y    ;
		msgControllerFullData[index].torque_z          = 0                           ; // r_ftData.wrench.torque.z    ;

		msgControllerFullData[index].flexiforce_1      = flexiForce(0)               ;
		msgControllerFullData[index].flexiforce_2      = flexiForce(1)               ;
		msgControllerFullData[index].flexiforce_3      = flexiForce(2)               ;
		msgControllerFullData[index].flexiforce_4      = flexiForce(3)               ;

		msgControllerFullData[index].FT_force_x        = FT_transformed_force(0)     ;
		msgControllerFullData[index].FT_force_y        = FT_transformed_force(1)     ;
		msgControllerFullData[index].FT_force_z        = FT_transformed_force(2)     ;

		msgControllerFullData[index].acc_x             = r_acc_data(0)               ;
		msgControllerFullData[index].acc_y             = r_acc_data(1)               ;
		msgControllerFullData[index].acc_z             = r_acc_data(2)               ;

		// Input reference efforts(torques)
		msgControllerFullData[index].reference_eff_j0  = 0                           ; //t_r(0) ;
		msgControllerFullData[index].reference_eff_j1  = 0                           ; //t_r(1) ;
		msgControllerFullData[index].reference_eff_j2  = 0                           ; //t_r(2) ;
		msgControllerFullData[index].reference_eff_j3  = 0                           ; //t_r(3) ;
		msgControllerFullData[index].reference_eff_j4  = 0                           ; //t_r(4) ;
		msgControllerFullData[index].reference_eff_j5  = 0                           ; //t_r(5) ;
		msgControllerFullData[index].reference_eff_j6  = 0                           ; //t_r(6) ;

		// Cartesian task reference
		msgControllerFullData[index].taskRef_x         = task_ref(0)                 ;
		msgControllerFullData[index].taskRef_y         = task_ref(1)                 ;
		msgControllerFullData[index].taskRef_z         = task_ref(2)                 ;
		msgControllerFullData[index].taskRef_phi       = 0                           ;
		msgControllerFullData[index].taskRef_theta     = 0                           ;
		msgControllerFullData[index].taskRef_psi       = 0                           ;

		// Cartesian task reference
		msgControllerFullData[index].taskRefModel_x    = task_refModel_output(0)            ;
		msgControllerFullData[index].taskRefModel_y    = task_refModel_output(1)            ;
		msgControllerFullData[index].taskRefModel_z    = task_refModel_output(2)            ;
		msgControllerFullData[index].taskRefModel_phi  = 0                           ;
		msgControllerFullData[index].taskRefModel_theta= 0                           ;
		msgControllerFullData[index].taskRefModel_psi  = 0                           ;

		// Model States
		msgControllerFullData[index].m_cartPos_x       = modelCartPos_.position.x    ;
		msgControllerFullData[index].m_cartPos_y       = modelCartPos_.position.y    ;
		msgControllerFullData[index].m_cartPos_z       = modelCartPos_.position.z    ;
		msgControllerFullData[index].m_cartPos_Qx      = modelCartPos_.orientation.x ;
		msgControllerFullData[index].m_cartPos_Qy      = modelCartPos_.orientation.y ;
		msgControllerFullData[index].m_cartPos_Qz      = modelCartPos_.orientation.z ;
		msgControllerFullData[index].m_cartPos_QW      = modelCartPos_.orientation.w ;

		msgControllerFullData[index].m_pos_x           = X_m(0)                      ;
		msgControllerFullData[index].m_pos_y           = X_m(1)                      ;
		msgControllerFullData[index].m_pos_z           = X_m(2)                      ;

		msgControllerFullData[index].m_vel_x           = Xd_m(0)                     ;
		msgControllerFullData[index].m_vel_y           = Xd_m(1)                     ;
		msgControllerFullData[index].m_vel_z           = Xd_m(2)                     ;

		msgControllerFullData[index].m_acc_x           = Xdd_m(0)                    ;
		msgControllerFullData[index].m_acc_y           = Xdd_m(1)                    ;
		msgControllerFullData[index].m_acc_z           = Xdd_m(2)                    ;

		msgControllerFullData[index].m_pos_j0          = q_m(0)                      ;
		msgControllerFullData[index].m_pos_j1          = q_m(1)                      ;
		msgControllerFullData[index].m_pos_j2          = q_m(2)                      ;
		msgControllerFullData[index].m_pos_j3          = q_m(3)                      ;
		msgControllerFullData[index].m_pos_j4          = q_m(4)                      ;
		msgControllerFullData[index].m_pos_j5          = q_m(5)                      ;
		msgControllerFullData[index].m_pos_j6          = q_m(6)                      ;

		msgControllerFullData[index].m_vel_j0          = qd_m(0)                     ;
		msgControllerFullData[index].m_vel_j1          = qd_m(1)                     ;
		msgControllerFullData[index].m_vel_j2          = qd_m(2)                     ;
		msgControllerFullData[index].m_vel_j3          = qd_m(3)                     ;
		msgControllerFullData[index].m_vel_j4          = qd_m(4)                     ;
		msgControllerFullData[index].m_vel_j5          = qd_m(5)                     ;
		msgControllerFullData[index].m_vel_j6          = qd_m(6)                     ;

		msgControllerFullData[index].m_acc_j0          = qdd_m(0)                    ;
		msgControllerFullData[index].m_acc_j1          = qdd_m(1)                    ;
		msgControllerFullData[index].m_acc_j2          = qdd_m(2)                    ;
		msgControllerFullData[index].m_acc_j3          = qdd_m(3)                    ;
		msgControllerFullData[index].m_acc_j4          = qdd_m(4)                    ;
		msgControllerFullData[index].m_acc_j5          = qdd_m(5)                    ;
		msgControllerFullData[index].m_acc_j6          = qdd_m(6)                    ;

		msgControllerFullData[index].m_eff_j0          = 0                           ;
		msgControllerFullData[index].m_eff_j1          = 0                           ;
		msgControllerFullData[index].m_eff_j2          = 0                           ;
		msgControllerFullData[index].m_eff_j3          = 0                           ;
		msgControllerFullData[index].m_eff_j4          = 0                           ;
		msgControllerFullData[index].m_eff_j5          = 0                           ;
		msgControllerFullData[index].m_eff_j6          = 0                           ;

		// Control Output
		msgControllerFullData[index].control_eff_j0    = tau(0)                      ;
		msgControllerFullData[index].control_eff_j1    = tau(1)                      ;
		msgControllerFullData[index].control_eff_j2    = tau(2)                      ;
		msgControllerFullData[index].control_eff_j3    = tau(3)                      ;
		msgControllerFullData[index].control_eff_j4    = tau(4)                      ;
		msgControllerFullData[index].control_eff_j5    = tau(5)                      ;
		msgControllerFullData[index].control_eff_j6    = tau(6)                      ;

		// Robot States
		msgControllerFullData[index].r_cartPos_x       = robotCartPos_.position.x    ;
		msgControllerFullData[index].r_cartPos_y       = robotCartPos_.position.y    ;
		msgControllerFullData[index].r_cartPos_z       = robotCartPos_.position.z    ;
		msgControllerFullData[index].r_cartPos_Qx      = robotCartPos_.orientation.x ;
		msgControllerFullData[index].r_cartPos_Qy      = robotCartPos_.orientation.y ;
		msgControllerFullData[index].r_cartPos_Qz      = robotCartPos_.orientation.z ;
		msgControllerFullData[index].r_cartPos_QW      = robotCartPos_.orientation.w ;

		msgControllerFullData[index].r_pos_j0          = q(0)                        ;
		msgControllerFullData[index].r_pos_j1          = q(1)                        ;
		msgControllerFullData[index].r_pos_j2          = q(2)                        ;
		msgControllerFullData[index].r_pos_j3          = q(3)                        ;
		msgControllerFullData[index].r_pos_j4          = q(4)                        ;
		msgControllerFullData[index].r_pos_j5          = q(5)                        ;
		msgControllerFullData[index].r_pos_j6          = q(6)                        ;

		msgControllerFullData[index].r_vel_j0          = qd(0)                       ;
		msgControllerFullData[index].r_vel_j1          = qd(1)                       ;
		msgControllerFullData[index].r_vel_j2          = qd(2)                       ;
		msgControllerFullData[index].r_vel_j3          = qd(3)                       ;
		msgControllerFullData[index].r_vel_j4          = qd(4)                       ;
		msgControllerFullData[index].r_vel_j5          = qd(5)                       ;
		msgControllerFullData[index].r_vel_j6          = qd(6)                       ;

		msgControllerFullData[index].r_acc_j0          = 0                           ;
		msgControllerFullData[index].r_acc_j1          = 0                           ;
		msgControllerFullData[index].r_acc_j2          = 0                           ;
		msgControllerFullData[index].r_acc_j3          = 0                           ;
		msgControllerFullData[index].r_acc_j4          = 0                           ;
		msgControllerFullData[index].r_acc_j5          = 0                           ;
		msgControllerFullData[index].r_acc_j6          = 0                           ;
		/*
          msgControllerFullData[index].r_eff_x           = ferr_(0)                    ;
          msgControllerFullData[index].r_eff_y           = ferr_(1)                    ;
          msgControllerFullData[index].r_eff_z           = ferr_(2)                    ;
          msgControllerFullData[index].r_trq_x           = ferr_(3)                    ;
          msgControllerFullData[index].r_trq_y           = ferr_(4)                    ;
          msgControllerFullData[index].r_trq_z           = ferr_(5)                    ;

          msgControllerFullData[index].r_eff_j0          = tau_f_(0)                   ;
          msgControllerFullData[index].r_eff_j1          = tau_f_(1)                   ;
          msgControllerFullData[index].r_eff_j2          = tau_f_(2)                   ;
          msgControllerFullData[index].r_eff_j3          = tau_f_(3)                   ;
          msgControllerFullData[index].r_eff_j4          = tau_f_(4)                   ;
          msgControllerFullData[index].r_eff_j5          = tau_f_(5)                   ;
          msgControllerFullData[index].r_eff_j6          = tau_f_(6)                   ;
		 */
		msgControllerFullData[index].l_limit_0         = q_lower(0)                  ;
		msgControllerFullData[index].l_limit_1         = q_lower(1)                  ;
		msgControllerFullData[index].l_limit_2         = q_lower(2)                  ;
		msgControllerFullData[index].l_limit_3         = q_lower(3)                  ;
		msgControllerFullData[index].l_limit_4         = q_lower(4)                  ;
		msgControllerFullData[index].l_limit_5         = q_lower(5)                  ;
		msgControllerFullData[index].l_limit_6         = q_lower(6)                  ;

		msgControllerFullData[index].u_limit_0         = q_upper(0)                  ;
		msgControllerFullData[index].u_limit_1         = q_upper(1)                  ;
		msgControllerFullData[index].u_limit_2         = q_upper(2)                  ;
		msgControllerFullData[index].u_limit_3         = q_upper(3)                  ;
		msgControllerFullData[index].u_limit_4         = q_upper(4)                  ;
		msgControllerFullData[index].u_limit_5         = q_upper(5)                  ;
		msgControllerFullData[index].u_limit_6         = q_upper(6)                  ;

		// NN Params
		msgControllerFullData[index].kappa             = kappa                       ;
		msgControllerFullData[index].Kv                = Kv                          ;
		msgControllerFullData[index].lambda            = lambda                      ;
		msgControllerFullData[index].Kz                = Kz                          ;
		msgControllerFullData[index].Zb                = Zb                          ;
		msgControllerFullData[index].F                 = nnF                         ;
		msgControllerFullData[index].G                 = nnG                         ;
		msgControllerFullData[index].inParams          = num_Inputs                  ;
		msgControllerFullData[index].outParams         = num_Outputs                 ;
		msgControllerFullData[index].hiddenNodes       = num_Hidden                  ;
		msgControllerFullData[index].errorParams       = num_Error                   ;
		msgControllerFullData[index].feedForwardForce  = num_Joints                  ;
		msgControllerFullData[index].nn_ON             = nn_ON                       ;

		// Cart params
		msgControllerFullData[index].cartPos_Kp_x      = cartPos_Kp_x                ;
		msgControllerFullData[index].cartPos_Kp_y      = cartPos_Kp_y                ;
		msgControllerFullData[index].cartPos_Kp_z      = cartPos_Kp_z                ;
		msgControllerFullData[index].cartPos_Kd_x      = cartPos_Kd_x                ;
		msgControllerFullData[index].cartPos_Kd_y      = cartPos_Kd_y                ;
		msgControllerFullData[index].cartPos_Kd_z      = cartPos_Kd_z                ;

		msgControllerFullData[index].cartRot_Kp_x      = cartRot_Kp_x                ;
		msgControllerFullData[index].cartRot_Kp_y      = cartRot_Kp_y                ;
		msgControllerFullData[index].cartRot_Kp_z      = cartRot_Kp_z                ;
		msgControllerFullData[index].cartRot_Kd_x      = cartRot_Kd_x                ;
		msgControllerFullData[index].cartRot_Kd_y      = cartRot_Kd_y                ;
		msgControllerFullData[index].cartRot_Kd_z      = cartRot_Kd_z                ;

		msgControllerFullData[index].useCurrentCartPose= useCurrentCartPose          ;
		msgControllerFullData[index].useNullspacePose  = useNullspacePose            ;

		msgControllerFullData[index].cartIniX          = cartIniX                    ;
		msgControllerFullData[index].cartIniY          = cartIniY                    ;
		msgControllerFullData[index].cartIniZ          = cartIniZ                    ;
		msgControllerFullData[index].cartIniRoll       = cartIniRoll                 ;
		msgControllerFullData[index].cartIniPitch      = cartIniPitch                ;
		msgControllerFullData[index].cartIniYaw        = cartIniYaw                  ;

		msgControllerFullData[index].cartDesX          = cartDesX                    ;
		msgControllerFullData[index].cartDesY          = cartDesY                    ;
		msgControllerFullData[index].cartDesZ          = cartDesZ                    ;
		msgControllerFullData[index].cartDesRoll       = cartDesRoll                 ;
		msgControllerFullData[index].cartDesPitch      = cartDesPitch                ;
		msgControllerFullData[index].cartDesYaw        = cartDesYaw                  ;

		// TODO fix this
		// Model Params
		// 2nd degree ref model
		msgControllerFullData[index].m                 = m_M                         ; // outerLoopMSDmodel.getMass(  )(0,0) ;
		msgControllerFullData[index].d                 = m_D                         ; // outerLoopMSDmodel.getSpring()(0,0) ;
		msgControllerFullData[index].k                 = m_S                         ; // outerLoopMSDmodel.getDamper()(0,0) ;
		// 1st degree ref model
		msgControllerFullData[index].task_mA           = task_mA                     ;
		msgControllerFullData[index].task_mB           = task_mB                     ;

		msgControllerFullData[index].fixedFilterWeights= useFixedWeights             ;

		msgControllerFullData[index].w0                = outerLoopWk(0,0)            ;
		msgControllerFullData[index].w1                = outerLoopWk(1,0)            ;
		msgControllerFullData[index].w2                = outerLoopWk(2,0)            ;
		msgControllerFullData[index].w3                = outerLoopWk(3,0)            ;
		msgControllerFullData[index].w4                = outerLoopWk(4,0)            ;
		msgControllerFullData[index].w5                = outerLoopWk(5,0)            ;
		msgControllerFullData[index].w6                = outerLoopWk(6,0)            ;
		msgControllerFullData[index].w7                = outerLoopWk(7,0)            ;

		msgControllerFullData[index].f1_w0             = outerLoopWk_flexi_1(0,0)    ;
		msgControllerFullData[index].f1_w1             = outerLoopWk_flexi_1(1,0)    ;
		msgControllerFullData[index].f1_w2             = outerLoopWk_flexi_1(2,0)    ;
		msgControllerFullData[index].f1_w3             = outerLoopWk_flexi_1(3,0)    ;
		msgControllerFullData[index].f1_w4             = outerLoopWk_flexi_1(4,0)    ;
		msgControllerFullData[index].f1_w5             = outerLoopWk_flexi_1(5,0)    ;
		msgControllerFullData[index].f1_w6             = outerLoopWk_flexi_1(6,0)    ;
		msgControllerFullData[index].f1_w7             = outerLoopWk_flexi_1(7,0)    ;

		msgControllerFullData[index].f2_w0             = outerLoopWk_flexi_2(0,0)    ;
		msgControllerFullData[index].f2_w1             = outerLoopWk_flexi_2(1,0)    ;
		msgControllerFullData[index].f2_w2             = outerLoopWk_flexi_2(2,0)    ;
		msgControllerFullData[index].f2_w3             = outerLoopWk_flexi_2(3,0)    ;
		msgControllerFullData[index].f2_w4             = outerLoopWk_flexi_2(4,0)    ;
		msgControllerFullData[index].f2_w5             = outerLoopWk_flexi_2(5,0)    ;
		msgControllerFullData[index].f2_w6             = outerLoopWk_flexi_2(6,0)    ;
		msgControllerFullData[index].f2_w7             = outerLoopWk_flexi_2(7,0)    ;

		msgControllerFullData[index].f3_w0             = outerLoopWk_flexi_3(0,0)    ;
		msgControllerFullData[index].f3_w1             = outerLoopWk_flexi_3(1,0)    ;
		msgControllerFullData[index].f3_w2             = outerLoopWk_flexi_3(2,0)    ;
		msgControllerFullData[index].f3_w3             = outerLoopWk_flexi_3(3,0)    ;
		msgControllerFullData[index].f3_w4             = outerLoopWk_flexi_3(4,0)    ;
		msgControllerFullData[index].f3_w5             = outerLoopWk_flexi_3(5,0)    ;
		msgControllerFullData[index].f3_w6             = outerLoopWk_flexi_3(6,0)    ;
		msgControllerFullData[index].f3_w7             = outerLoopWk_flexi_3(7,0)    ;

		msgControllerFullData[index].f4_w0             = outerLoopWk_flexi_4(0,0)    ;
		msgControllerFullData[index].f4_w1             = outerLoopWk_flexi_4(1,0)    ;
		msgControllerFullData[index].f4_w2             = outerLoopWk_flexi_4(2,0)    ;
		msgControllerFullData[index].f4_w3             = outerLoopWk_flexi_4(3,0)    ;
		msgControllerFullData[index].f4_w4             = outerLoopWk_flexi_4(4,0)    ;
		msgControllerFullData[index].f4_w5             = outerLoopWk_flexi_4(5,0)    ;
		msgControllerFullData[index].f4_w6             = outerLoopWk_flexi_4(6,0)    ;
		msgControllerFullData[index].f4_w7             = outerLoopWk_flexi_4(7,0)    ;

		// MRAC Params
		outerLoopMRACmodelY.getGamma( msgControllerFullData[index].gamma_1,
				msgControllerFullData[index].gamma_2,
				msgControllerFullData[index].gamma_3,
				msgControllerFullData[index].gamma_4,
				msgControllerFullData[index].gamma_5 );

		outerLoopMRACmodelY.getEstimatedParams( msgControllerFullData[index].y_hat  ,
				msgControllerFullData[index].theta_1,
				msgControllerFullData[index].theta_2,
				msgControllerFullData[index].theta_3,
				msgControllerFullData[index].theta_4,
				msgControllerFullData[index].ahat   ,
				msgControllerFullData[index].bhat    );

		// Increment for the next cycle.
		storage_index_ = index+1;

		/*          // Add a data point
          setDataPoint(controllerData.add_datum(), dt);*/
	}

	/*        // Save to file
        {
              if (!controllerData.SerializeToOstream(&saveDataFile))
              {
                    ROS_ERROR_STREAM( "Failed to write data." );
              }
        }

            // Optional:  Delete all global objects allocated by libprotobuf.
            google::protobuf::ShutdownProtobufLibrary();*/
}

/*
void PR2CartneuroControllerClass::setDataPoint(dataPoint::Datum* datum, double & dt)
{
	datum->set_dt                 (  dt                          );
	datum->set_force_x            (  (double)transformed_force(0));
	datum->set_force_y            (  (double)transformed_force(1));
	datum->set_force_z            (  (double)transformed_force(2));
	datum->set_torque_x           (  0                           );
	datum->set_torque_y           (  0                           );
	datum->set_torque_z           (  0                           );
	datum->set_acc_x              (  (double)acc_data(0)         );
	datum->set_acc_y              (  (double)acc_data(1)         );
	datum->set_acc_z              (  (double)acc_data(2)         );
	datum->set_reference_eff_j0   (  0                           );
	datum->set_reference_eff_j1   (  0                           );
	datum->set_reference_eff_j2   (  0                           );
	datum->set_reference_eff_j3   (  0                           );
	datum->set_reference_eff_j4   (  0                           );
	datum->set_reference_eff_j5   (  0                           );
	datum->set_reference_eff_j6   (  0                           );
	datum->set_taskref_x          (  (double)task_ref(0)         );
	datum->set_taskref_y          (  (double)task_ref(1)         );
	datum->set_taskref_z          (  (double)task_ref(2)         );
	datum->set_taskref_phi        (  0                           );
	datum->set_taskref_theta      (  0                           );
	datum->set_taskref_psi        (  0                           );
	datum->set_taskrefmodel_x     (  (double)task_refModel(0)    );
	datum->set_taskrefmodel_y     (  (double)task_refModel(1)    );
	datum->set_taskrefmodel_z     (  (double)task_refModel(2)    );
	datum->set_taskrefmodel_phi   (  0                           );
	datum->set_taskrefmodel_theta (  0                           );
	datum->set_taskrefmodel_psi   (  0                           );
	datum->set_m_cartpos_x        (  modelCartPos_.position.x    );
	datum->set_m_cartpos_y        (  modelCartPos_.position.y    );
	datum->set_m_cartpos_z        (  modelCartPos_.position.z    );
	datum->set_m_cartpos_qx       (  modelCartPos_.orientation.x );
	datum->set_m_cartpos_qy       (  modelCartPos_.orientation.y );
	datum->set_m_cartpos_qz       (  modelCartPos_.orientation.z );
	datum->set_m_cartpos_qw       (  modelCartPos_.orientation.w );
	datum->set_m_pos_x            (  (double)X_m(0)              );
	datum->set_m_pos_y            (  (double)X_m(1)              );
	datum->set_m_pos_z            (  (double)X_m(2)              );
	datum->set_m_vel_x            (  (double)Xd_m(0)             );
	datum->set_m_vel_y            (  (double)Xd_m(1)             );
	datum->set_m_vel_z            (  (double)Xd_m(2)             );
	datum->set_m_acc_x            (  (double)Xdd_m(0)            );
	datum->set_m_acc_y            (  (double)Xdd_m(1)            );
	datum->set_m_acc_z            (  (double)Xdd_m(2)            );
	datum->set_m_pos_j0           (  (double)q_m(0)              );
	datum->set_m_pos_j1           (  (double)q_m(1)              );
	datum->set_m_pos_j2           (  (double)q_m(2)              );
	datum->set_m_pos_j3           (  (double)q_m(3)              );
	datum->set_m_pos_j4           (  (double)q_m(4)              );
	datum->set_m_pos_j5           (  (double)q_m(5)              );
	datum->set_m_pos_j6           (  (double)q_m(6)              );
	datum->set_m_vel_j0           (  (double)qd_m(0)             );
	datum->set_m_vel_j1           (  (double)qd_m(1)             );
	datum->set_m_vel_j2           (  (double)qd_m(2)             );
	datum->set_m_vel_j3           (  (double)qd_m(3)             );
	datum->set_m_vel_j4           (  (double)qd_m(4)             );
	datum->set_m_vel_j5           (  (double)qd_m(5)             );
	datum->set_m_vel_j6           (  (double)qd_m(6)             );
	datum->set_m_acc_j0           (  (double)qdd_m(0)            );
	datum->set_m_acc_j1           (  (double)qdd_m(1)            );
	datum->set_m_acc_j2           (  (double)qdd_m(2)            );
	datum->set_m_acc_j3           (  (double)qdd_m(3)            );
	datum->set_m_acc_j4           (  (double)qdd_m(4)            );
	datum->set_m_acc_j5           (  (double)qdd_m(5)            );
	datum->set_m_acc_j6           (  (double)qdd_m(6)            );
	datum->set_m_eff_j0           (  0                           );
	datum->set_m_eff_j1           (  0                           );
	datum->set_m_eff_j2           (  0                           );
	datum->set_m_eff_j3           (  0                           );
	datum->set_m_eff_j4           (  0                           );
	datum->set_m_eff_j5           (  0                           );
	datum->set_m_eff_j6           (  0                           );
	datum->set_control_eff_j0     (  (double)tau(0)              );
	datum->set_control_eff_j1     (  (double)tau(1)              );
	datum->set_control_eff_j2     (  (double)tau(2)              );
	datum->set_control_eff_j3     (  (double)tau(3)              );
	datum->set_control_eff_j4     (  (double)tau(4)              );
	datum->set_control_eff_j5     (  (double)tau(5)              );
	datum->set_control_eff_j6     (  (double)tau(6)              );
	datum->set_r_cartpos_x        (  robotCartPos_.position.x    );
	datum->set_r_cartpos_y        (  robotCartPos_.position.y    );
	datum->set_r_cartpos_z        (  robotCartPos_.position.z    );
	datum->set_r_cartpos_qx       (  robotCartPos_.orientation.x );
	datum->set_r_cartpos_qy       (  robotCartPos_.orientation.y );
	datum->set_r_cartpos_qz       (  robotCartPos_.orientation.z );
	datum->set_r_cartpos_qw       (  robotCartPos_.orientation.w );
	datum->set_r_pos_j0           (  (double)q(0)                );
	datum->set_r_pos_j1           (  (double)q(1)                );
	datum->set_r_pos_j2           (  (double)q(2)                );
	datum->set_r_pos_j3           (  (double)q(3)                );
	datum->set_r_pos_j4           (  (double)q(4)                );
	datum->set_r_pos_j5           (  (double)q(5)                );
	datum->set_r_pos_j6           (  (double)q(6)                );
	datum->set_r_vel_j0           (  (double)qd(0)               );
	datum->set_r_vel_j1           (  (double)qd(1)               );
	datum->set_r_vel_j2           (  (double)qd(2)               );
	datum->set_r_vel_j3           (  (double)qd(3)               );
	datum->set_r_vel_j4           (  (double)qd(4)               );
	datum->set_r_vel_j5           (  (double)qd(5)               );
	datum->set_r_vel_j6           (  (double)qd(6)               );
	datum->set_r_acc_j0           (  0                           );
	datum->set_r_acc_j1           (  0                           );
	datum->set_r_acc_j2           (  0                           );
	datum->set_r_acc_j3           (  0                           );
	datum->set_r_acc_j4           (  0                           );
	datum->set_r_acc_j5           (  0                           );
	datum->set_r_acc_j6           (  0                           );
//      datum->set_r_eff_x            (  ferr_(0)                    );
//	datum->set_r_eff_y            (  ferr_(1)                    );
//	datum->set_r_eff_z            (  ferr_(2)                    );
//	datum->set_r_trq_x            (  ferr_(3)                    );
//	datum->set_r_trq_y            (  ferr_(4)                    );
//	datum->set_r_trq_z            (  ferr_(5)                    );
//	datum->set_r_eff_j0           (  tau_f_(0)                   );
//	datum->set_r_eff_j1           (  tau_f_(1)                   );
//	datum->set_r_eff_j2           (  tau_f_(2)                   );
//	datum->set_r_eff_j3           (  tau_f_(3)                   );
//	datum->set_r_eff_j4           (  tau_f_(4)                   );
//	datum->set_r_eff_j5           (  tau_f_(5)                   );
//	datum->set_r_eff_j6           (  tau_f_(6)                   );
	datum->set_l_limit_0          (  q_lower(0)                  );
	datum->set_l_limit_1          (  q_lower(1)                  );
	datum->set_l_limit_2          (  q_lower(2)                  );
	datum->set_l_limit_3          (  q_lower(3)                  );
	datum->set_l_limit_4          (  q_lower(4)                  );
	datum->set_l_limit_5          (  q_lower(5)                  );
	datum->set_l_limit_6          (  q_lower(6)                  );
	datum->set_u_limit_0          (  q_upper(0)                  );
	datum->set_u_limit_1          (  q_upper(1)                  );
	datum->set_u_limit_2          (  q_upper(2)                  );
	datum->set_u_limit_3          (  q_upper(3)                  );
	datum->set_u_limit_4          (  q_upper(4)                  );
	datum->set_u_limit_5          (  q_upper(5)                  );
	datum->set_u_limit_6          (  q_upper(6)                  );
	datum->set_kappa              (  kappa                       );
	datum->set_kv                 (  Kv                          );
	datum->set_lambda             (  lambda                      );
	datum->set_kz                 (  Kz                          );
	datum->set_zb                 (  Zb                          );
	datum->set_f                  (  nnF                         );
	datum->set_g                  (  nnG                         );
	datum->set_inparams           (  num_Inputs                  );
	datum->set_outparams          (  num_Outputs                 );
	datum->set_hiddennodes        (  num_Hidden                  );
	datum->set_errorparams        (  num_Error                   );
	datum->set_feedforwardforce   (  num_Joints                  );
	datum->set_nn_on              (  nn_ON                       );
	datum->set_cartpos_kp_x       (  cartPos_Kp_x                );
	datum->set_cartpos_kp_y       (  cartPos_Kp_y                );
	datum->set_cartpos_kp_z       (  cartPos_Kp_z                );
	datum->set_cartpos_kd_x       (  cartPos_Kd_x                );
	datum->set_cartpos_kd_y       (  cartPos_Kd_y                );
	datum->set_cartpos_kd_z       (  cartPos_Kd_z                );
	datum->set_cartrot_kp_x       (  cartRot_Kp_x                );
	datum->set_cartrot_kp_y       (  cartRot_Kp_y                );
	datum->set_cartrot_kp_z       (  cartRot_Kp_z                );
	datum->set_cartrot_kd_x       (  cartRot_Kd_x                );
	datum->set_cartrot_kd_y       (  cartRot_Kd_y                );
	datum->set_cartrot_kd_z       (  cartRot_Kd_z                );
	datum->set_usecurrentcartpose (  useCurrentCartPose          );
	datum->set_usenullspacepose   (  useNullspacePose            );
	datum->set_cartinix           (  cartIniX                    );
	datum->set_cartiniy           (  cartIniY                    );
	datum->set_cartiniz           (  cartIniZ                    );
	datum->set_cartiniroll        (  cartIniRoll                 );
	datum->set_cartinipitch       (  cartIniPitch                );
	datum->set_cartiniyaw         (  cartIniYaw                  );
	datum->set_cartdesx           (  cartDesX                    );
	datum->set_cartdesy           (  cartDesY                    );
	datum->set_cartdesz           (  cartDesZ                    );
	datum->set_cartdesroll        (  cartDesRoll                 );
	datum->set_cartdespitch       (  cartDesPitch                );
	datum->set_cartdesyaw         (  cartDesYaw                  );
	datum->set_m                  (  m_M                         );
	datum->set_d                  (  m_D                         );
	datum->set_k                  (  m_S                         );
	datum->set_task_ma            (  task_mA                     );
	datum->set_task_mb            (  task_mB                     );
	datum->set_fixedfilterweights (  fixedFilterWeights          );
	datum->set_w0                 (  (double)outerLoopWk(0,0)    );
	datum->set_w1                 (  (double)outerLoopWk(1,0)    );
	datum->set_w2                 (  (double)outerLoopWk(2,0)    );
	datum->set_w3                 (  (double)outerLoopWk(3,0)    );
	datum->set_w4                 (  (double)outerLoopWk(4,0)    );
	datum->set_w5                 (  (double)outerLoopWk(5,0)    );
	datum->set_w6                 (  (double)outerLoopWk(6,0)    );
	datum->set_w7                 (  (double)outerLoopWk(7,0)    );

}
 */


/// Service call to set reference trajectory
bool PR2CartneuroControllerClass::setRefTraj( ice_msgs::setCartPose::Request  & req ,
                                                    ice_msgs::setCartPose::Response & resp )
{
	if( externalRefTraj )
	{
		task_ref(0) = req.msg.position.x ;
		task_ref(1) = req.msg.position.y ;
		task_ref(2) = req.msg.position.z ;
	}
	resp.success = true;
	return true;
}

/// Service call to capture and extract the data
bool PR2CartneuroControllerClass::paramUpdate( ice_msgs::controllerParamUpdate::Request  & req ,
                                                     ice_msgs::controllerParamUpdate::Response & resp )
{

	num_Inputs  = req.msg.inParams                 ;
	num_Outputs = req.msg.outParams                ;
	num_Hidden  = req.msg.hiddenNodes              ;
	num_Error   = req.msg.errorParams              ;

	kappa       = req.msg.kappa                    ;
	Kv          = req.msg.Kv                       ;
	lambda      = req.msg.lambda                   ;
	Kz          = req.msg.Kz                       ;
	Zb          = req.msg.Zb                       ;
	fFForce     = req.msg.feedForwardForce         ;
	nn_ON       = req.msg.nn_ON                    ;
	nnF         = req.msg.F                        ;
	nnG         = req.msg.G                        ;

	// FIXME actually update these in the
	// outer-loop controllers
	m_M         = req.msg.m                        ;
	m_D         = req.msg.d                        ;
	m_S         = req.msg.k                        ;

	task_mA     = req.msg.task_mA                  ;
	task_mB     = req.msg.task_mB                  ;

	// Cart params
	cartPos_Kp_x      = req.msg.cartPos_Kp_x       ;
	cartPos_Kp_y      = req.msg.cartPos_Kp_y       ;
	cartPos_Kp_z      = req.msg.cartPos_Kp_z       ;
	cartPos_Kd_x      = req.msg.cartPos_Kd_x       ;
	cartPos_Kd_y      = req.msg.cartPos_Kd_y       ;
	cartPos_Kd_z      = req.msg.cartPos_Kd_z       ;

	cartRot_Kp_x      = req.msg.cartRot_Kp_x       ;
	cartRot_Kp_y      = req.msg.cartRot_Kp_y       ;
	cartRot_Kp_z      = req.msg.cartRot_Kp_z       ;
	cartRot_Kd_x      = req.msg.cartRot_Kd_x       ;
	cartRot_Kd_y      = req.msg.cartRot_Kd_y       ;
	cartRot_Kd_z      = req.msg.cartRot_Kd_z       ;

	useCurrentCartPose= req.msg.useCurrentCartPose ;
	useNullspacePose  = req.msg.useNullspacePose   ;

	cartIniX          = req.msg.cartIniX           ;
	cartIniY          = req.msg.cartIniY           ;
	cartIniZ          = req.msg.cartIniZ           ;
	cartIniRoll       = req.msg.cartIniRoll        ;
	cartIniPitch      = req.msg.cartIniPitch       ;
	cartIniYaw        = req.msg.cartIniYaw         ;

	cartDesX          = req.msg.cartDesX           ;
	cartDesY          = req.msg.cartDesY           ;
	cartDesZ          = req.msg.cartDesZ           ;
	cartDesRoll       = req.msg.cartDesRoll        ;
	cartDesPitch      = req.msg.cartDesPitch       ;
	cartDesYaw        = req.msg.cartDesYaw         ;

	nnController.changeNNstructure( num_Inputs  ,   // num_Inputs
                                    num_Outputs ,   // num_Outputs
                                    num_Hidden  ,   // num_Hidden
                                    num_Error   ,   // num_Error
                                    num_Joints   ); // num_Joints

	Eigen::MatrixXd p_Kv     ;
	Eigen::MatrixXd p_lambda ;

	p_Kv     .resize( 6, 1 ) ;
	p_lambda .resize( 6, 1 ) ;

	p_Kv << cartPos_Kd_x ,
			cartPos_Kd_y ,
			cartPos_Kd_z ,
			cartRot_Kd_x ,
			cartRot_Kd_y ,
			cartRot_Kd_z ;

	p_lambda << cartPos_Kp_x / cartPos_Kd_x ,
			cartPos_Kp_y / cartPos_Kd_y ,
			cartPos_Kp_z / cartPos_Kd_z ,
			cartRot_Kp_x / cartRot_Kd_x ,
			cartRot_Kp_y / cartRot_Kd_y ,
			cartRot_Kp_z / cartRot_Kd_z ;

	nnController.init( kappa    ,
			p_Kv     ,
			p_lambda ,
			Kz       ,
			Zb       ,
			fFForce  ,
			nnF      ,
			nnG      ,
			nn_ON     );

	// MRAC
	outerLoopMRACmodelX.updateAB( task_mA,
                                  task_mB );
	outerLoopMRACmodelY.updateAB( task_mA,
                                  task_mB );

	// RLS
	outerLoopRLSmodelX.updateAB( task_mA,
                                 task_mB );
	outerLoopRLSmodelY.updateAB( task_mA,
                                 task_mB );

	// MSD
	outerLoopMSDmodelX.updateMsd( m_M,
                                  m_S,
                                  m_D );
	outerLoopMSDmodelY.updateMsd( m_M,
                                  m_S,
                                  m_D );

	// IRL
	outerLoopIRLmodelX.updateMsd( m_M,
                                  m_S,
                                  m_D );
	outerLoopIRLmodelY.updateMsd( m_M,
                                  m_S,
                                  m_D );

	resp.success = true;

	return true;
}

/// Service call to save the data
bool PR2CartneuroControllerClass::save( ice_msgs::saveControllerData::Request & req,
                                             ice_msgs::saveControllerData::Response& resp )
{
	/* Record the starting time. */
	ros::Time started = ros::Time::now();

	// Start circle traj
	startCircleTraj = true;

	/* Mark the buffer as clear (which will start storing). */
	storage_index_ = 0;

	std::string fileNameProto = req.fileName ;

	// Save data to file
	saveDataFile.open(fileNameProto.c_str(), std::ios::out | std::ios::trunc | std::ios::binary);

	resp.success = true;

	return true;
}

/// Service call to publish the saved data
bool PR2CartneuroControllerClass::publish( std_srvs::Empty::Request & req,
                                                std_srvs::Empty::Response& resp )
{
	/* Then we can publish the buffer contents. */
	int  index;
	for (index = 0 ; index < StoreLen ; index++)
	{
		//    pubFTData_         .publish(msgFTData         [index]);
		//    pubModelStates_    .publish(msgModelStates    [index]);
		//    pubRobotStates_    .publish(msgRobotStates    [index]);
		//    pubModelCartPos_   .publish(msgModelCartPos   [index]);
		//    pubRobotCartPos_   .publish(msgRobotCartPos   [index]);
		//    pubControllerParam_.publish(msgControllerParam[index]);
		pubControllerFullData_.publish(msgControllerFullData[index]);
	}

	return true;
}


/// Service call to capture and extract the data
bool PR2CartneuroControllerClass::capture(	std_srvs::Empty::Request & req,
                                                std_srvs::Empty::Response& resp )
{
	/* Record the starting time. */
	ros::Time started = ros::Time::now();

	// Start circle traj
	startCircleTraj = true;

	/* Mark the buffer as clear (which will start storing). */
	storage_index_ = 0;

	/* Now wait until the buffer is full. */
	while (storage_index_ < StoreLen)
	{
		/* Sleep for 1ms as not to hog the CPU. */
		ros::Duration(0.001).sleep();

		/* Make sure we don't hang here forever. */
		if (ros::Time::now() - started > ros::Duration(20))
		{
			ROS_ERROR("Waiting for buffer to fill up took longer than 20 seconds!");
			return false;
		}
	}

	// Start circle traj
	startCircleTraj = false;

	/* Then we can publish the buffer contents. */
	int  index;
	for (index = 0 ; index < StoreLen ; index++)
	{
		//    pubFTData_         .publish(msgFTData         [index]);
		//    pubModelStates_    .publish(msgModelStates    [index]);
		//    pubRobotStates_    .publish(msgRobotStates    [index]);
		//    pubModelCartPos_   .publish(msgModelCartPos   [index]);
		//    pubRobotCartPos_   .publish(msgRobotCartPos   [index]);
		//    pubControllerParam_.publish(msgControllerParam[index]);
		pubControllerFullData_.publish(msgControllerFullData[index]);
	}

	return true;
}

/// Service call to set reference trajectory
bool PR2CartneuroControllerClass::toggleFixedWeights( ice_msgs::fixedWeightToggle::Request & req,
		ice_msgs::fixedWeightToggle::Response& resp )
{
	// XOR to toggle
	useFixedWeights =  ( useFixedWeights || true ) && !( useFixedWeights && true );
	resp.useFixedWeights = useFixedWeights ;
	// Filter Weights
	resp.w0 = filtW0 ; // outerLoopWk(0,0) ;
	resp.w1 = filtW1 ; // outerLoopWk(1,0) ;
	resp.w2 = filtW2 ; // outerLoopWk(2,0) ;
	resp.w3 = filtW3 ; // outerLoopWk(3,0) ;
	resp.w4 = filtW4 ; // outerLoopWk(4,0) ;
	resp.w5 = filtW5 ; // outerLoopWk(5,0) ;
	resp.w6 = filtW6 ; // outerLoopWk(6,0) ;
	resp.w7 = filtW7 ; // outerLoopWk(7,0) ;

	outerLoopWk(0,0) = filtW0 ;
	outerLoopWk(1,0) = filtW1 ;
	outerLoopWk(2,0) = filtW2 ;
	outerLoopWk(3,0) = filtW3 ;
	outerLoopWk(4,0) = filtW4 ;
	outerLoopWk(5,0) = filtW5 ;
	outerLoopWk(6,0) = filtW6 ;
	outerLoopWk(7,0) = filtW7 ;

	if( useFixedWeights )
	{
		outerLoopRLSmodelY.setFixedWeights( outerLoopWk );
	}else
	{
		outerLoopRLSmodelY.setUpdatedWeights();
	}

	std::vector<std::string> outerModel ;

	if( useARMAmodel   ){ outerModel.push_back("useARMAmodel"  ); }
	if( useCTARMAmodel ){ outerModel.push_back("useCTARMAmodel"); }
	if( useFIRmodel    ){ outerModel.push_back("useFIRmodel "  ); }
	if( useMRACmodel   ){ outerModel.push_back("useMRACmodel"  ); }
	if( useMSDmodel    ){ outerModel.push_back("useMSDmodel "  ); }

	if( useIRLmodel  )
	{
		if( useFixedWeights )
		{
			outerModel.push_back("useIRLmodel : Update");
			outerLoopIRLmodelY.setUpdateIrl();
		}else
		{
			outerModel.push_back("useIRLmodel : Fixed");
			outerLoopIRLmodelY.setFixedMsd();
		}
	}

	resp.outerModel = outerModel;

	return true;
}

/// Controller stopping in realtime
void PR2CartneuroControllerClass::stopping()
{}

void
PR2CartneuroControllerClass::calcHumanIntentPos( Eigen::Vector3d & force,
		Eigen::VectorXd & pos,
		double delT,
		double m )
{
	Eigen::Vector3d intentPos = Eigen::Vector3d::Zero();
	Eigen::Vector3d intentVel = Eigen::Vector3d::Zero();
	Eigen::Vector3d intentAcc = Eigen::Vector3d::Zero();

	Eigen::Vector3d M(m, m, m);

	intentAcc = force.cwiseQuotient(M);

	intentVel = intentVel + intentAcc * delT ;
	intentPos = intentPos + intentVel * delT ;

	pos = intentPos;
}

void PR2CartneuroControllerClass::readForceValuesCB(const geometry_msgs::WrenchConstPtr& wrench_msg)
{
	// convert to wrench command
	flexiforce_wrench_desi_.force(0) = wrench_msg->force.x;
	flexiforce_wrench_desi_.force(1) = wrench_msg->force.y;
	flexiforce_wrench_desi_.force(2) = wrench_msg->force.z;
	flexiforce_wrench_desi_.torque(0) = wrench_msg->torque.x;
	flexiforce_wrench_desi_.torque(1) = wrench_msg->torque.y;
	flexiforce_wrench_desi_.torque(2) = wrench_msg->torque.z;
}

bool PR2CartneuroControllerClass::initParam()
{

	nh_.param("/forceTorqueOn",     forceTorqueOn,     false);
	nh_.param("/accelerometerOn",   accelerometerOn,   false);
	nh_.param("/useFlexiForce",     useFlexiForce,     false);
	nh_.param("/executeCircleTraj", executeCircleTraj, false);
	nh_.param("/publishRTtopics",   publishRTtopics, false);

	return true;
}

bool PR2CartneuroControllerClass::initRobot()
{
	// Get the root and tip link names from parameter server.
	if (!nh_.getParam("root_name", root_name))
	{
		ROS_ERROR("No root name given in namespace: %s)",nh_.getNamespace().c_str());
		return false;
	}
	if (!nh_.getParam("tip_name", tip_name))
	{
		ROS_ERROR("No tip name given in namespace: %s)",nh_.getNamespace().c_str());
		return false;
	}

	// Construct a chain from the root to the tip and prepare the kinematics.
	// Note the joints must be calibrated.
	if (!chain_.init(robot_state_, root_name, tip_name))
	{
		ROS_ERROR("The Cartesian Controller could not use the chain from '%s' to '%s'", root_name.c_str(), tip_name.c_str());
		return false;
	}

	// Get gripper accelerometer tip name and construct a chain
	std::string gripper_acc_tip;
	if (!nh_.getParam("/gripper_acc_tip", gripper_acc_tip))
	{
		ROS_ERROR("No accelerometer tip name given in namespace: %s)",nh_.getNamespace().c_str());
		return false;
	}
	if (!chain_acc_link.init(robot_state_, root_name, gripper_acc_tip))
	{
		ROS_ERROR("MyCartController could not use the chain from '%s' to '%s'", root_name.c_str(), gripper_acc_tip.c_str());
		return false;
	}

	// Load urdf model
	std::string urdf_param_ = "/robot_description";
	std::string urdf_string;

	if (!nh_.getParam(urdf_param_, urdf_string))
	{
		ROS_ERROR("URDF not loaded from parameter: %s)", urdf_param_.c_str());
		return false;
	}

	if (!urdf_model.initString(urdf_string))
	{
		ROS_ERROR("Failed to parse URDF file");
		return false;
	}else {
		ROS_INFO("Successfully parsed URDF file");
	}

	// Construct the kdl solvers in non-realtime.
	chain_.toKDL(kdl_chain_);
	chain_acc_link.toKDL(kdl_chain_acc_link);

	kin_.reset(new Kin<Joints>(kdl_chain_));
	kin_acc_.reset(new Kin<Joints>(kdl_chain_acc_link));

	// Resize (pre-allocate) the variables in non-realtime.
	tau_c_.resize(kdl_chain_.getNrOfJoints());						// TODO relpace kdl_chain_.getNrOfJoints() with Joints
	tau_measured_.resize(kdl_chain_.getNrOfJoints());

	qnom.resize(kdl_chain_.getNrOfJoints());
	q_lower.resize(kdl_chain_.getNrOfJoints());
	q_upper.resize(kdl_chain_.getNrOfJoints());
	qd_limit.resize(kdl_chain_.getNrOfJoints());

	q_lower(0) = urdf_model.getJoint("l_shoulder_pan_joint"  )->limits->lower;
	q_lower(1) = urdf_model.getJoint("l_shoulder_lift_joint" )->limits->lower;
	q_lower(2) = urdf_model.getJoint("l_upper_arm_roll_joint")->limits->lower;
	q_lower(3) = urdf_model.getJoint("l_elbow_flex_joint"    )->limits->lower;
	q_lower(4) = urdf_model.getJoint("l_forearm_roll_joint"  )->limits->lower;
	q_lower(5) = urdf_model.getJoint("l_wrist_flex_joint"    )->limits->lower;
	q_lower(6) = urdf_model.getJoint("l_wrist_roll_joint"    )->limits->lower;

	q_upper(0) = urdf_model.getJoint("l_shoulder_pan_joint"  )->limits->upper;
	q_upper(1) = urdf_model.getJoint("l_shoulder_lift_joint" )->limits->upper;
	q_upper(2) = urdf_model.getJoint("l_upper_arm_roll_joint")->limits->upper;
	q_upper(3) = urdf_model.getJoint("l_elbow_flex_joint"    )->limits->upper;
	q_upper(4) = urdf_model.getJoint("l_forearm_roll_joint"  )->limits->upper;
	q_upper(5) = urdf_model.getJoint("l_wrist_flex_joint"    )->limits->upper;
	q_upper(6) = urdf_model.getJoint("l_wrist_roll_joint"    )->limits->upper;

	// Since two joints are continuous TODO describe why?
	q_upper(4) =   6.28 ;
	q_upper(6) =   6.28 ;

	q_lower(4) = - 6.28 ;
	q_lower(6) = - 6.28 ;

	qnom(0) = ( q_upper(0) - q_lower(0) ) / 2 ;
	qnom(1) = ( q_upper(1) - q_lower(1) ) / 2 ;
	qnom(2) = ( q_upper(2) - q_lower(2) ) / 2 ;
	qnom(3) = ( q_upper(3) - q_lower(3) ) / 2 ;
	qnom(4) = ( q_upper(4) - q_lower(4) ) / 2 ;
	qnom(5) = ( q_upper(5) - q_lower(5) ) / 2 ;
	qnom(6) = ( q_upper(6) - q_lower(6) ) / 2 ;

	// Load Cartesian gains
	std::string para_cartPos_Kp_x = "/cartPos_Kp_x";
	std::string para_cartPos_Kp_y = "/cartPos_Kp_y";
	std::string para_cartPos_Kp_z = "/cartPos_Kp_z";
	std::string para_cartPos_Kd_x = "/cartPos_Kd_x";
	std::string para_cartPos_Kd_y = "/cartPos_Kd_y";
	std::string para_cartPos_Kd_z = "/cartPos_Kd_z";

	std::string para_cartRot_Kp_x = "/cartRot_Kp_x";
	std::string para_cartRot_Kp_y = "/cartRot_Kp_y";
	std::string para_cartRot_Kp_z = "/cartRot_Kp_z";
	std::string para_cartRot_Kd_x = "/cartRot_Kd_x";
	std::string para_cartRot_Kd_y = "/cartRot_Kd_y";
	std::string para_cartRot_Kd_z = "/cartRot_Kd_z";

	cartPos_Kp_x = 0 ; cartRot_Kp_x = 0 ;
	cartPos_Kp_y = 0 ; cartRot_Kp_y = 0 ;
	cartPos_Kp_z = 0 ; cartRot_Kp_z = 0 ;
	cartPos_Kd_x = 0 ; cartRot_Kd_x = 0 ;
	cartPos_Kd_y = 0 ; cartRot_Kd_y = 0 ;
	cartPos_Kd_z = 0 ; cartRot_Kd_z = 0 ;

	if (!nh_.getParam( para_cartPos_Kp_x , cartPos_Kp_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kp_x.c_str()) ; return false; }
	if (!nh_.getParam( para_cartPos_Kp_y , cartPos_Kp_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kp_y.c_str()) ; return false; }
	if (!nh_.getParam( para_cartPos_Kp_z , cartPos_Kp_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kp_z.c_str()) ; return false; }
	if (!nh_.getParam( para_cartPos_Kd_x , cartPos_Kd_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kd_x.c_str()) ; return false; }
	if (!nh_.getParam( para_cartPos_Kd_y , cartPos_Kd_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kd_y.c_str()) ; return false; }
	if (!nh_.getParam( para_cartPos_Kd_z , cartPos_Kd_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kd_z.c_str()) ; return false; }

	if (!nh_.getParam( para_cartRot_Kp_x , cartRot_Kp_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kp_x.c_str()) ; return false; }
	if (!nh_.getParam( para_cartRot_Kp_y , cartRot_Kp_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kp_y.c_str()) ; return false; }
	if (!nh_.getParam( para_cartRot_Kp_z , cartRot_Kp_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kp_z.c_str()) ; return false; }
	if (!nh_.getParam( para_cartRot_Kd_x , cartRot_Kd_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kd_x.c_str()) ; return false; }
	if (!nh_.getParam( para_cartRot_Kd_y , cartRot_Kd_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kd_y.c_str()) ; return false; }
	if (!nh_.getParam( para_cartRot_Kd_z , cartRot_Kd_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kd_z.c_str()) ; return false; }


	// Posture control
	if (!nh_.getParam( "/k_posture", k_posture)){ ROS_ERROR("Failed to load parameter: %s !)", "/k_posture" ) ;  k_posture = 1.0; }
	if (!nh_.getParam( "/jacobian_inverse_damping", k_posture)){ ROS_ERROR("Failed to load parameter: %s !)", "/jacobian_inverse_damping" ) ;  jacobian_inverse_damping = 0.0; }

	for (int i = 0; i < Joints; ++i)
	{
		std::string tmp = chain_.getJoint(i)->joint_->name;

		if (!nh_.getParam( "/joint_feedforward/" + tmp, joint_dd_ff_[i] )){ ROS_ERROR("Failed to load /joint_feedforward!"); joint_dd_ff_[i]=0.0; }
		if (!nh_.getParam( "/posture/" + tmp, 			q_posture_[i]   )){ ROS_ERROR("Failed to load /posture!");           q_posture_[i]=0.0; }
		if (!nh_.getParam( "/saturation/" + tmp, 		saturation_[i]  )){ ROS_ERROR("Failed to load /saturation!"); 		 saturation_[i]=0.0; }
	}

	// Get FT frame ID
	ft_frame_id = root_name;
	if (!nh_.getParam("/ft_frame_id", ft_frame_id))
	{
		ROS_ERROR("No ft_frame_id name given in namespace: %s)",nh_.getNamespace().c_str());
		return false;
	}

	// Construct kdl solvers for F/T frames
	if(forceTorqueOn)
	{
		if (!chain_ft_link.init(robot_state_, root_name, ft_frame_id))
		{
			ROS_ERROR("MyCartController could not use the chain from '%s' to '%s'", root_name.c_str(), ft_frame_id.c_str());
			return false;
		}
		chain_ft_link.toKDL(kdl_chain_ft_link);
		kin_ft_.reset(new Kin<Joints>(kdl_chain_ft_link));
	}

	return true;
}

bool PR2CartneuroControllerClass::initTrajectories()
{

	// Rate for circle trajectory
	circle_rate = 3  ;
	circleLlim  = 0  ;
	circleUlim  = 1.5;

	std::string para_circleRate  = "/circleRate" ;
	std::string para_circleLlim  = "/circleLlim" ;
	std::string para_circleUlim  = "/circleUlim" ;

	if (!nh_.getParam( para_circleRate , circle_rate )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_circleRate .c_str()) ; return false; }
	if (!nh_.getParam( para_circleLlim , circleLlim  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_circleLlim .c_str()) ; return false; }
	if (!nh_.getParam( para_circleUlim , circleUlim  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_circleUlim .c_str()) ; return false; }


	// Desired cartesian pose
	cartDesX     = 0.0 ;
	cartDesY     = 0.0 ;
	cartDesZ     = 0.0 ;
	cartDesRoll  = 0.0 ;
	cartDesPitch = 0.0 ;
	cartDesYaw   = 0.0 ;

	std::string para_cartDesX     = "/cartDesX";
	std::string para_cartDesY     = "/cartDesY";
	std::string para_cartDesZ     = "/cartDesZ";
	std::string para_cartDesRoll  = "/cartDesRoll";
	std::string para_cartDesPitch = "/cartDesPitch";
	std::string para_cartDesYaw   = "/cartDesYaw";

	if (!nh_.getParam( para_cartDesX     , cartDesX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesX    .c_str()) ; return false; }
	if (!nh_.getParam( para_cartDesY     , cartDesY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesY    .c_str()) ; return false; }
	if (!nh_.getParam( para_cartDesZ     , cartDesZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesZ    .c_str()) ; return false; }
	if (!nh_.getParam( para_cartDesRoll  , cartDesRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesRoll .c_str()) ; return false; }
	if (!nh_.getParam( para_cartDesPitch , cartDesPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesPitch.c_str()) ; return false; }
	if (!nh_.getParam( para_cartDesYaw   , cartDesYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesYaw  .c_str()) ; return false; }


	// Initial cartesian pose
	cartIniX     = 0.7 ;
	cartIniY     = 0.3 ;
	cartIniZ     = 0.1 ;
	cartIniRoll  = 0.0 ;
	cartIniPitch = 0.0 ;
	cartIniYaw   = 0.0 ;

	std::string para_cartIniX     = "/cartIniX";
	std::string para_cartIniY     = "/cartIniY";
	std::string para_cartIniZ     = "/cartIniZ";
	std::string para_cartIniRoll  = "/cartIniRoll";
	std::string para_cartIniPitch = "/cartIniPitch";
	std::string para_cartIniYaw   = "/cartIniYaw";

	if (!nh_.getParam( para_cartIniX     , cartIniX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniX.c_str()) ; return false; }
	if (!nh_.getParam( para_cartIniY     , cartIniY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniY.c_str()) ; return false; }
	if (!nh_.getParam( para_cartIniZ     , cartIniZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniZ.c_str()) ; return false; }
	if (!nh_.getParam( para_cartIniRoll  , cartIniRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniRoll .c_str()) ; return false; }
	if (!nh_.getParam( para_cartIniPitch , cartIniPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniPitch.c_str()) ; return false; }
	if (!nh_.getParam( para_cartIniYaw   , cartIniYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniYaw  .c_str()) ; return false; }


	return true;
}

bool PR2CartneuroControllerClass::initSensors()
{
	bool result=true;

	// Get a handle to the hardware interface
	pr2_hardware_interface::HardwareInterface* hardwareInterface = robot_state_->model_->hw_;
	if(!hardwareInterface)
		ROS_ERROR("Something wrong with the hardware interface pointer!");

	// Get F/T handles
	if( forceTorqueOn )
	{
		l_ft_handle_ = hardwareInterface->getForceTorque("l_gripper_motor");
		r_ft_handle_ = hardwareInterface->getForceTorque("r_gripper_motor");

		if( !l_ft_handle_ )
		{
			ROS_ERROR("Something wrong with getting l_ft handle");
			result=false;
		}
		if( !r_ft_handle_ )
		{
			ROS_ERROR("Something wrong with getting r_ft handle");
			result=false;
		}
	}

	// Get accelerometer handles
	if(accelerometerOn)
	{
		/* get a handle to the left gripper accelerometer */
		l_accelerometer_handle_ = hardwareInterface->getAccelerometer("l_gripper_motor");
		if(!l_accelerometer_handle_)
		{
			ROS_ERROR("Something wrong with getting accelerometer handle");
			result=false;
		}

		// set to 1.5 kHz bandwidth (should be the default)
		l_accelerometer_handle_->command_.bandwidth_ = pr2_hardware_interface::AccelerometerCommand
				::BANDWIDTH_1500HZ;

		// set to +/- 8g range (0=2g,1=4g)
		l_accelerometer_handle_->command_.range_ = pr2_hardware_interface::AccelerometerCommand
				::RANGE_8G;

		/* get a handle to the right gripper accelerometer */
		r_accelerometer_handle_ = hardwareInterface->getAccelerometer("r_gripper_motor");
		if(!r_accelerometer_handle_)
		{
			ROS_ERROR("Something wrong with getting accelerometer handle");
			result=false;
		}

		// set to 1.5 kHz bandwidth (should be the default)
		r_accelerometer_handle_->command_.bandwidth_ = pr2_hardware_interface::AccelerometerCommand
				::BANDWIDTH_1500HZ;

		// set to +/- 8g range (0=2g,1=4g)
		r_accelerometer_handle_->command_.range_ = pr2_hardware_interface::AccelerometerCommand
				::RANGE_4G;

		l_accelerationObserver = new accelerationObserver(l_accelerometer_handle_);
		r_accelerationObserver = new accelerationObserver(r_accelerometer_handle_);
	}

	return result;
}

bool PR2CartneuroControllerClass::initNN()
{

	nnController.changeNNstructure( num_Inputs  ,   // num_Inputs
                                    num_Outputs ,   // num_Outputs
                                    num_Hidden  ,   // num_Hidden
                                    num_Error   ,   // num_Error
                                    num_Outputs );  // num_Joints = num_Outputs for cart space

	Eigen::MatrixXd p_Kv     ;
	Eigen::MatrixXd p_lambda ;

	p_Kv     .resize( num_Outputs, 1 ) ;
	p_lambda .resize( num_Outputs, 1 ) ;

	// Filtered error
	// r = (qd_m - qd) + lambda*(q_m - q);
	// Kv*r
	// Kd*(qd_m - qd) + Kp*(q_m - q) = Kv*(qd_m - qd) + Kv*lambda*(q_m - q);
	// Kv = Kd | Kv*lambda = Kp ... lambda = Kp/Kv = Kp/Kd

	p_Kv << cartPos_Kd_x ,
            cartPos_Kd_y ,
            cartPos_Kd_z ,
            cartRot_Kd_x ,
            cartRot_Kd_y ,
            cartRot_Kd_z ;

	p_lambda << cartPos_Kp_x / cartPos_Kd_x ,
                cartPos_Kp_y / cartPos_Kd_y ,
                cartPos_Kp_z / cartPos_Kd_z ,
                cartRot_Kp_x / cartRot_Kd_x ,
                cartRot_Kp_y / cartRot_Kd_y ,
                cartRot_Kp_z / cartRot_Kd_z ;

	nnController.init( kappa  ,
                       p_Kv     ,
                       p_lambda ,
                       Kz     ,
                       Zb     ,
                       fFForce,
                       nnF    ,
                       nnG    ,
                       nn_ON   );

	nnController.updateDelT( delT );

	return true;
}


bool PR2CartneuroControllerClass::initOuterLoop()
{
	bool result=true;


	std::string para_m_M = "/m_M" ;
	std::string para_m_S = "/m_S" ;
	std::string para_m_D = "/m_D" ;

	if (!nh_.getParam( para_m_M , m_M )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_M .c_str()) ; return false; }
	if (!nh_.getParam( para_m_S , m_S )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_S .c_str()) ; return false; }
	if (!nh_.getParam( para_m_D , m_D )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_m_D .c_str()) ; return false; }

	std::string para_task_mA = "/task_mA" ;
	std::string para_task_mB = "/task_mB" ;

	if (!nh_.getParam( para_task_mA , task_mA )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_task_mA .c_str()) ; return false; }
	if (!nh_.getParam( para_task_mB , task_mB )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_task_mB .c_str()) ; return false; }



	useCurrentCartPose = false ;
	std::string para_useCurrentCartPose     = "/useCurrentCartPose";
	if (!nh_.getParam( para_useCurrentCartPose, useCurrentCartPose )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useCurrentCartPose.c_str()) ; return false; }

	useNullspacePose = true ;
	std::string para_useNullspacePose     = "/useNullspacePose";
	if (!nh_.getParam( para_useNullspacePose, useNullspacePose )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useNullspacePose.c_str()) ; return false; }

	useARMAmodel = false ;
	std::string para_useARMAmodel = "/useARMAmodel";
	if (!nh_.getParam( para_useARMAmodel, useARMAmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useARMAmodel.c_str()) ; return false; }

	useCTARMAmodel = false ;
	std::string para_useCTARMAmodel = "/useCTARMAmodel";
	if (!nh_.getParam( para_useCTARMAmodel, useCTARMAmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useCTARMAmodel.c_str()) ; return false; }

	useFIRmodel = false ;
	std::string para_useFIRmodel = "/useFIRmodel";
	if (!nh_.getParam( para_useFIRmodel, useFIRmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useFIRmodel.c_str()) ; return false; }

	useMRACmodel = false ;
	std::string para_useMRACmodel = "/useMRACmodel";
	if (!nh_.getParam( para_useMRACmodel, useMRACmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useMRACmodel.c_str()) ; return false; }

	useMSDmodel = false ;
	std::string para_useMSDmodel = "/useMSDmodel";
	if (!nh_.getParam( para_useMSDmodel, useMSDmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useMSDmodel.c_str()) ; return false; }

	useIRLmodel = false ;
	std::string para_useIRLmodel = "/useIRLmodel";
	if (!nh_.getParam( para_useIRLmodel, useIRLmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useIRLmodel.c_str()) ; return false; }

	useDirectmodel = false ;
	std::string para_useDirectmodel = "/useDirectmodel";
	if (!nh_.getParam( para_useDirectmodel, useDirectmodel )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useDirectmodel.c_str()) ; return false; }


	externalRefTraj = true ;
	std::string para_externalRefTraj = "/externalRefTraj";
	if (!nh_.getParam( para_externalRefTraj, externalRefTraj )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_externalRefTraj.c_str()) ; return false; }

	intentEst_delT = 0.1 ;
	std::string para_intentEst_delT = "/intentEst_delT";
	if (!nh_.getParam( para_intentEst_delT, intentEst_delT )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_intentEst_delT.c_str()) ; return false; }

	intentEst_M = 1.0 ;
	std::string para_intentEst_M = "/intentEst_M";
	if (!nh_.getParam( para_intentEst_M, intentEst_M )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_intentEst_M.c_str()) ; return false; }




	std::string para_forceCutOffX = "/forceCutOffX";
	std::string para_forceCutOffY = "/forceCutOffY";
	std::string para_forceCutOffZ = "/forceCutOffZ";

	if (!nh_.getParam( para_forceCutOffX , forceCutOffX )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffX.c_str()) ; return false; }
	if (!nh_.getParam( para_forceCutOffY , forceCutOffY )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffY.c_str()) ; return false; }
	if (!nh_.getParam( para_forceCutOffZ , forceCutOffZ )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_forceCutOffZ.c_str()) ; return false; }



	std::string para_filtW0        = "/filtW0"        ;
	std::string para_filtW1        = "/filtW1"        ;
	std::string para_filtW2        = "/filtW2"        ;
	std::string para_filtW3        = "/filtW3"        ;
	std::string para_filtW4        = "/filtW4"        ;
	std::string para_filtW5        = "/filtW5"        ;
	std::string para_filtW6        = "/filtW6"        ;
	std::string para_filtW7        = "/filtW7"        ;

	std::string para_flex_1_filtW0 = "/flex_1_filtW0" ;
	std::string para_flex_1_filtW1 = "/flex_1_filtW1" ;
	std::string para_flex_1_filtW2 = "/flex_1_filtW2" ;
	std::string para_flex_1_filtW3 = "/flex_1_filtW3" ;
	std::string para_flex_1_filtW4 = "/flex_1_filtW4" ;
	std::string para_flex_1_filtW5 = "/flex_1_filtW5" ;
	std::string para_flex_1_filtW6 = "/flex_1_filtW6" ;
	std::string para_flex_1_filtW7 = "/flex_1_filtW7" ;

	std::string para_flex_2_filtW0 = "/flex_2_filtW0" ;
	std::string para_flex_2_filtW1 = "/flex_2_filtW1" ;
	std::string para_flex_2_filtW2 = "/flex_2_filtW2" ;
	std::string para_flex_2_filtW3 = "/flex_2_filtW3" ;
	std::string para_flex_2_filtW4 = "/flex_2_filtW4" ;
	std::string para_flex_2_filtW5 = "/flex_2_filtW5" ;
	std::string para_flex_2_filtW6 = "/flex_2_filtW6" ;
	std::string para_flex_2_filtW7 = "/flex_2_filtW7" ;

	std::string para_flex_3_filtW0 = "/flex_3_filtW0" ;
	std::string para_flex_3_filtW1 = "/flex_3_filtW1" ;
	std::string para_flex_3_filtW2 = "/flex_3_filtW2" ;
	std::string para_flex_3_filtW3 = "/flex_3_filtW3" ;
	std::string para_flex_3_filtW4 = "/flex_3_filtW4" ;
	std::string para_flex_3_filtW5 = "/flex_3_filtW5" ;
	std::string para_flex_3_filtW6 = "/flex_3_filtW6" ;
	std::string para_flex_3_filtW7 = "/flex_3_filtW7" ;

	std::string para_flex_4_filtW0 = "/flex_4_filtW0" ;
	std::string para_flex_4_filtW1 = "/flex_4_filtW1" ;
	std::string para_flex_4_filtW2 = "/flex_4_filtW2" ;
	std::string para_flex_4_filtW3 = "/flex_4_filtW3" ;
	std::string para_flex_4_filtW4 = "/flex_4_filtW4" ;
	std::string para_flex_4_filtW5 = "/flex_4_filtW5" ;
	std::string para_flex_4_filtW6 = "/flex_4_filtW6" ;
	std::string para_flex_4_filtW7 = "/flex_4_filtW7" ;

	filtW0 = 0.0 ;  flex_1_filtW0 = 0.0 ;  flex_2_filtW0 = 0.0 ;  flex_3_filtW0 = 0.0 ;  flex_4_filtW0 = 0.0 ;
	filtW1 = 0.0 ;  flex_1_filtW1 = 0.0 ;  flex_2_filtW1 = 0.0 ;  flex_3_filtW1 = 0.0 ;  flex_4_filtW1 = 0.0 ;
	filtW2 = 0.0 ;  flex_1_filtW2 = 0.0 ;  flex_2_filtW2 = 0.0 ;  flex_3_filtW2 = 0.0 ;  flex_4_filtW2 = 0.0 ;
	filtW3 = 0.0 ;  flex_1_filtW3 = 0.0 ;  flex_2_filtW3 = 0.0 ;  flex_3_filtW3 = 0.0 ;  flex_4_filtW3 = 0.0 ;
	filtW4 = 0.0 ;  flex_1_filtW4 = 0.0 ;  flex_2_filtW4 = 0.0 ;  flex_3_filtW4 = 0.0 ;  flex_4_filtW4 = 0.0 ;
	filtW5 = 0.0 ;  flex_1_filtW5 = 0.0 ;  flex_2_filtW5 = 0.0 ;  flex_3_filtW5 = 0.0 ;  flex_4_filtW5 = 0.0 ;
	filtW6 = 0.0 ;  flex_1_filtW6 = 0.0 ;  flex_2_filtW6 = 0.0 ;  flex_3_filtW6 = 0.0 ;  flex_4_filtW6 = 0.0 ;
	filtW7 = 0.0 ;  flex_1_filtW7 = 0.0 ;  flex_2_filtW7 = 0.0 ;  flex_3_filtW7 = 0.0 ;  flex_4_filtW7 = 0.0 ;

	if (!nh_.getParam( para_filtW0        , filtW0        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW0       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW1        , filtW1        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW1       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW2        , filtW2        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW2       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW3        , filtW3        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW3       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW4        , filtW4        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW4       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW5        , filtW5        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW5       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW6        , filtW6        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW6       .c_str()) ; return false; }
	if (!nh_.getParam( para_filtW7        , filtW7        )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_filtW7       .c_str()) ; return false; }

	if (!nh_.getParam( para_flex_1_filtW0 , flex_1_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW0.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW1 , flex_1_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW1.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW2 , flex_1_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW2.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW3 , flex_1_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW3.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW4 , flex_1_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW4.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW5 , flex_1_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW5.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW6 , flex_1_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW6.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_1_filtW7 , flex_1_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_1_filtW7.c_str()) ; return false; }

	if (!nh_.getParam( para_flex_2_filtW0 , flex_2_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW0.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW1 , flex_2_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW1.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW2 , flex_2_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW2.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW3 , flex_2_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW3.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW4 , flex_2_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW4.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW5 , flex_2_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW5.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW6 , flex_2_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW6.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_2_filtW7 , flex_2_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_2_filtW7.c_str()) ; return false; }

	if (!nh_.getParam( para_flex_3_filtW0 , flex_3_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW0.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW1 , flex_3_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW1.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW2 , flex_3_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW2.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW3 , flex_3_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW3.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW4 , flex_3_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW4.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW5 , flex_3_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW5.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW6 , flex_3_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW6.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_3_filtW7 , flex_3_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_3_filtW7.c_str()) ; return false; }

	if (!nh_.getParam( para_flex_4_filtW0 , flex_4_filtW0 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW0.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW1 , flex_4_filtW1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW1.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW2 , flex_4_filtW2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW2.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW3 , flex_4_filtW3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW3.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW4 , flex_4_filtW4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW4.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW5 , flex_4_filtW5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW5.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW6 , flex_4_filtW6 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW6.c_str()) ; return false; }
	if (!nh_.getParam( para_flex_4_filtW7 , flex_4_filtW7 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_flex_4_filtW7.c_str()) ; return false; }


	outerLoopWk.resize(8,1);
	outerLoopWk_flexi_1.resize(8,1);
	outerLoopWk_flexi_2.resize(8,1);
	outerLoopWk_flexi_3.resize(8,1);
	outerLoopWk_flexi_4.resize(8,1);


	if( useFIRmodel || useARMAmodel || useCTARMAmodel)
	{
		outerLoopWk(0,0) = filtW0 ; outerLoopWk_flexi_1(0,0) = flex_1_filtW0 ; outerLoopWk_flexi_2(0,0) = flex_2_filtW0 ; outerLoopWk_flexi_3(0,0) = flex_3_filtW0 ; outerLoopWk_flexi_4(0,0) = flex_4_filtW0 ;
		outerLoopWk(1,0) = filtW1 ; outerLoopWk_flexi_1(1,0) = flex_1_filtW1 ; outerLoopWk_flexi_2(1,0) = flex_2_filtW1 ; outerLoopWk_flexi_3(1,0) = flex_3_filtW1 ; outerLoopWk_flexi_4(1,0) = flex_4_filtW1 ;
		outerLoopWk(2,0) = filtW2 ; outerLoopWk_flexi_1(2,0) = flex_1_filtW2 ; outerLoopWk_flexi_2(2,0) = flex_2_filtW2 ; outerLoopWk_flexi_3(2,0) = flex_3_filtW2 ; outerLoopWk_flexi_4(2,0) = flex_4_filtW2 ;
		outerLoopWk(3,0) = filtW3 ; outerLoopWk_flexi_1(3,0) = flex_1_filtW3 ; outerLoopWk_flexi_2(3,0) = flex_2_filtW3 ; outerLoopWk_flexi_3(3,0) = flex_3_filtW3 ; outerLoopWk_flexi_4(3,0) = flex_4_filtW3 ;
		outerLoopWk(4,0) = filtW4 ; outerLoopWk_flexi_1(4,0) = flex_1_filtW4 ; outerLoopWk_flexi_2(4,0) = flex_2_filtW4 ; outerLoopWk_flexi_3(4,0) = flex_3_filtW4 ; outerLoopWk_flexi_4(4,0) = flex_4_filtW4 ;
		outerLoopWk(5,0) = filtW5 ; outerLoopWk_flexi_1(5,0) = flex_1_filtW5 ; outerLoopWk_flexi_2(5,0) = flex_2_filtW5 ; outerLoopWk_flexi_3(5,0) = flex_3_filtW5 ; outerLoopWk_flexi_4(5,0) = flex_4_filtW5 ;
		outerLoopWk(6,0) = filtW6 ; outerLoopWk_flexi_1(6,0) = flex_1_filtW6 ; outerLoopWk_flexi_2(6,0) = flex_2_filtW6 ; outerLoopWk_flexi_3(6,0) = flex_3_filtW6 ; outerLoopWk_flexi_4(6,0) = flex_4_filtW6 ;
		outerLoopWk(7,0) = filtW7 ; outerLoopWk_flexi_1(7,0) = flex_1_filtW7 ; outerLoopWk_flexi_2(7,0) = flex_2_filtW7 ; outerLoopWk_flexi_3(7,0) = flex_3_filtW7 ; outerLoopWk_flexi_4(7,0) = flex_4_filtW7 ;
	}else
	{
		outerLoopWk(0,0) = 0.0 ; outerLoopWk_flexi_1(0,0) = 0.0 ; outerLoopWk_flexi_2(0,0) = 0.0 ; outerLoopWk_flexi_3(0,0) = 0.0 ; outerLoopWk_flexi_4(0,0) = 0.0 ;
		outerLoopWk(1,0) = 0.0 ; outerLoopWk_flexi_1(1,0) = 0.0 ; outerLoopWk_flexi_2(1,0) = 0.0 ; outerLoopWk_flexi_3(1,0) = 0.0 ; outerLoopWk_flexi_4(1,0) = 0.0 ;
		outerLoopWk(2,0) = 0.0 ; outerLoopWk_flexi_1(2,0) = 0.0 ; outerLoopWk_flexi_2(2,0) = 0.0 ; outerLoopWk_flexi_3(2,0) = 0.0 ; outerLoopWk_flexi_4(2,0) = 0.0 ;
		outerLoopWk(3,0) = 0.0 ; outerLoopWk_flexi_1(3,0) = 0.0 ; outerLoopWk_flexi_2(3,0) = 0.0 ; outerLoopWk_flexi_3(3,0) = 0.0 ; outerLoopWk_flexi_4(3,0) = 0.0 ;
		outerLoopWk(4,0) = 0.0 ; outerLoopWk_flexi_1(4,0) = 0.0 ; outerLoopWk_flexi_2(4,0) = 0.0 ; outerLoopWk_flexi_3(4,0) = 0.0 ; outerLoopWk_flexi_4(4,0) = 0.0 ;
		outerLoopWk(5,0) = 0.0 ; outerLoopWk_flexi_1(5,0) = 0.0 ; outerLoopWk_flexi_2(5,0) = 0.0 ; outerLoopWk_flexi_3(5,0) = 0.0 ; outerLoopWk_flexi_4(5,0) = 0.0 ;
		outerLoopWk(6,0) = 0.0 ; outerLoopWk_flexi_1(6,0) = 0.0 ; outerLoopWk_flexi_2(6,0) = 0.0 ; outerLoopWk_flexi_3(6,0) = 0.0 ; outerLoopWk_flexi_4(6,0) = 0.0 ;
		outerLoopWk(7,0) = 0.0 ; outerLoopWk_flexi_1(7,0) = 0.0 ; outerLoopWk_flexi_2(7,0) = 0.0 ; outerLoopWk_flexi_3(7,0) = 0.0 ; outerLoopWk_flexi_4(7,0) = 0.0 ;
	}

	int numIrlSamples = 100;
	std::string para_numIrlSamples = "/numIrlSamples";
	if (!nh_.getParam( para_numIrlSamples , numIrlSamples )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_numIrlSamples.c_str()) ; return false; }

	int numIrlLsIter = 10;
	std::string para_numIrlLsIter = "/numIrlLsIter";
	if (!nh_.getParam( para_numIrlLsIter , numIrlLsIter )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_numIrlLsIter.c_str()) ; return false; }

	int numCartDof = 1;
	std::string para_numCartDof = "/numCartDof";
	if (!nh_.getParam( para_numCartDof , numCartDof )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_numCartDof.c_str()) ; return false; }

	bool irlOneshot = true;
	std::string para_irlOneshot = "/irlOneshot";
	if (!nh_.getParam( para_irlOneshot , irlOneshot )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_irlOneshot.c_str()) ; return false; }


	std::string para_fixedFilterWeights = "/fixedFilterWeights";
	if (!nh_.getParam( para_fixedFilterWeights , useFixedWeights )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_fixedFilterWeights.c_str()) ; return false; }

	double rls_lambda = 0.98 ;
	double rls_sigma  = 1000 ;

	std::string para_rls_lambda = "/rls_lambda";
	std::string para_rls_sigma  = "/rls_sigma";

	if (!nh_.getParam( para_rls_lambda , rls_lambda )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_rls_lambda.c_str()) ; return false; }
	if (!nh_.getParam( para_rls_sigma  , rls_sigma  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_rls_sigma .c_str()) ; return false; }

	double mrac_gamma_1 = 1 ;
	double mrac_gamma_2 = 1 ;
	double mrac_gamma_3 = 1 ;
	double mrac_gamma_4 = 1 ;
	double mrac_gamma_5 = 1 ;

	std::string para_mrac_gamma_1 = "/mrac_gamma_1";
	std::string para_mrac_gamma_2 = "/mrac_gamma_2";
	std::string para_mrac_gamma_3 = "/mrac_gamma_3";
	std::string para_mrac_gamma_4 = "/mrac_gamma_4";
	std::string para_mrac_gamma_5 = "/mrac_gamma_5";

	if (!nh_.getParam( para_mrac_gamma_1 , mrac_gamma_1 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_1.c_str()) ; return false; }
	if (!nh_.getParam( para_mrac_gamma_2 , mrac_gamma_2 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_2.c_str()) ; return false; }
	if (!nh_.getParam( para_mrac_gamma_3 , mrac_gamma_3 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_3.c_str()) ; return false; }
	if (!nh_.getParam( para_mrac_gamma_4 , mrac_gamma_4 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_4.c_str()) ; return false; }
	if (!nh_.getParam( para_mrac_gamma_5 , mrac_gamma_5 )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_gamma_5.c_str()) ; return false; }

	double mrac_P_m = 1 ;
	double mrac_P_h = 1 ;

	std::string para_mrac_P_m = "/mrac_P_m";
	std::string para_mrac_P_h = "/mrac_P_h";

	if (!nh_.getParam( para_mrac_P_m , mrac_P_m )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_P_m.c_str()) ; return false; }
	if (!nh_.getParam( para_mrac_P_h , mrac_P_h )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_mrac_P_h.c_str()) ; return false; }

	delT = 0.001;

	std::string para_outerLoopTime = "/outerLoop_time";
	if (!nh_.getParam( para_outerLoopTime , outerLoopTime )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_outerLoopTime.c_str()) ; return false; }

	intentLoopTime = outerLoopTime;
	std::string para_intentLoopTime = "/intentEst_time";
	if (!nh_.getParam( para_intentLoopTime , intentLoopTime )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_intentLoopTime.c_str()) ; return false; }

	useSimHuman = false ;
	std::string para_simHuman = "/useSimHuman";
	if (!nh_.getParam( para_simHuman , useSimHuman )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_simHuman.c_str()) ; return false; }

	std::string para_simHuman_a = "/simHuman_a" ;
	std::string para_simHuman_b = "/simHuman_b" ;

	if (!nh_.getParam( para_simHuman_a , simHuman_a )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_simHuman_a .c_str()) ; return false; }
	if (!nh_.getParam( para_simHuman_b , simHuman_b )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_simHuman_b .c_str()) ; return false; }

	// initial conditions
	ode_init_x[0 ] = 0.0;
	ode_init_x[1 ] = 0.0;
	ode_init_x[2 ] = 0.0;
	ode_init_x[3 ] = 0.0;

	/////////////////////////
	// System Model

	// FIXME remove below stuff
	num_Inputs  = 44 ;
	num_Outputs = 6  ; // 6 for 6 cart dof
	//  num_Hidden  = 100;
	num_Error   = 6  ;
	num_Joints  = 7  ;

	kdl_temp_joint_.resize( num_Joints );
	eigen_temp_joint.resize( num_Joints,1 );

	q       .resize( num_Joints ) ;
	qd      .resize( num_Joints ) ;
	qdd     .resize( num_Joints ) ;

	q_m     .resize( num_Outputs ) ;
	qd_m    .resize( num_Outputs ) ;
	qdd_m   .resize( num_Outputs ) ;

	// desired Cartesian states
	X_m     .resize( num_Outputs ) ;
	Xd_m    .resize( num_Outputs ) ;
	Xdd_m   .resize( num_Outputs ) ;

	// Prev desired Cartesian states
	p_X_m     .resize( num_Outputs ) ;
	p_Xd_m    .resize( num_Outputs ) ;
	p_Xdd_m   .resize( num_Outputs ) ;

	// Cartesian states
	X       .resize( num_Outputs ) ;
	Xd      .resize( num_Outputs ) ;

	t_r     .resize( num_Outputs ) ;
	task_ref.resize( num_Outputs ) ;
	task_refModel_output.resize( num_Outputs ) ;
	tau     .resize( num_Outputs ) ;

	q        = Eigen::VectorXd::Zero( num_Joints ) ;
	qd       = Eigen::VectorXd::Zero( num_Joints ) ;
	qdd      = Eigen::VectorXd::Zero( num_Joints ) ;
	q_m      = Eigen::VectorXd::Zero( num_Joints ) ;
	qd_m     = Eigen::VectorXd::Zero( num_Joints ) ;
	qdd_m    = Eigen::VectorXd::Zero( num_Joints ) ;

	X_m      = Eigen::VectorXd::Zero( num_Outputs ) ;
	Xd_m     = Eigen::VectorXd::Zero( num_Outputs ) ;
	Xdd_m    = Eigen::VectorXd::Zero( num_Outputs ) ;
	X        = Eigen::VectorXd::Zero( num_Outputs ) ;
	Xd       = Eigen::VectorXd::Zero( num_Outputs ) ;

	p_X_m    = Eigen::VectorXd::Zero( num_Outputs ) ;
	p_Xd_m   = Eigen::VectorXd::Zero( num_Outputs ) ;
	p_Xdd_m  = Eigen::VectorXd::Zero( num_Outputs ) ;

	X_m(0)   = cartIniX     ;
	X_m(1)   = cartIniY     ;
	X_m(2)   = cartIniZ     ;
	X_m(3)   = cartIniRoll  ;
	X_m(4)   = cartIniPitch ;
	X_m(5)   = cartIniYaw   ;

	p_X_m    = X_m   ;
	p_Xd_m   = Xd_m  ;
	p_Xdd_m  = Xdd_m ;

	transformed_force = Eigen::Vector3d::Zero();
	r_acc_data          = Eigen::Vector3d::Zero();

	t_r                  = Eigen::VectorXd::Zero( num_Outputs ) ;
	task_ref             = Eigen::VectorXd::Zero( num_Outputs ) ;
	task_refModel_output = Eigen::VectorXd::Zero( num_Outputs ) ;
	tau                  = Eigen::VectorXd::Zero( num_Outputs ) ;
	force_c              = Eigen::VectorXd::Zero( num_Outputs ) ;
	// FIXME remove this hardcoded 4 value
	flexiForce           = Eigen::VectorXd::Zero( 4 ) ;

	// Initial Reference
	task_ref = X_m ;

	JacobianPinv     = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), 6 ) ;
	JacobianTrans    = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), 6 ) ;
	JacobianTransPinv= Eigen::MatrixXd::Zero( num_Outputs, kdl_chain_.getNrOfJoints() ) ;
	nullSpace        = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints() ) ;

	cartControlForce = Eigen::VectorXd::Zero( num_Outputs ) ;
	nullspaceTorque  = Eigen::VectorXd::Zero( kdl_chain_.getNrOfJoints() ) ;
	controlTorque    = Eigen::VectorXd::Zero( kdl_chain_.getNrOfJoints() ) ;

	/////////////////////////
	// Outer Loop Init

	// MRAC
	outerLoopMRACmodelX.updateDelT( outerLoopTime );
	outerLoopMRACmodelX.updateAB( task_mA,
			task_mB );
	outerLoopMRACmodelX.updateIni( cartIniX,
			cartIniX );
	outerLoopMRACmodelX.updateSimHuman( useSimHuman );
	outerLoopMRACmodelX.updateSimHuman( simHuman_a,
			simHuman_b );
	outerLoopMRACmodelX.updateGamma( mrac_gamma_1,
			mrac_gamma_2,
			mrac_gamma_3,
			mrac_gamma_4,
			mrac_gamma_5 ) ;
	outerLoopMRACmodelX.updateCov( mrac_P_m,
			mrac_P_h ) ;

	outerLoopMRACmodelY.updateDelT( outerLoopTime );
	outerLoopMRACmodelY.updateAB( task_mA,
			task_mB );
	outerLoopMRACmodelY.updateIni( cartIniY,
			cartIniY );
	outerLoopMRACmodelY.updateSimHuman( useSimHuman );
	outerLoopMRACmodelY.updateSimHuman( simHuman_a,
			simHuman_b );
	outerLoopMRACmodelY.updateGamma( mrac_gamma_1,
			mrac_gamma_2,
			mrac_gamma_3,
			mrac_gamma_4,
			mrac_gamma_5 ) ;
	outerLoopMRACmodelY.updateCov( mrac_P_m,
			mrac_P_h ) ;

	// RLS

	outerLoopRLSmodelX.updateDelT( outerLoopTime );
	outerLoopRLSmodelX.updateAB( task_mA,
			task_mB );
	outerLoopRLSmodelX.initRls( rls_lambda, rls_sigma );
	//  outerLoopRLSmodelX.initPos( cartIniX );


	outerLoopRLSmodelY.updateDelT( outerLoopTime );
	outerLoopRLSmodelY.updateAB( task_mA,
			task_mB );
	outerLoopRLSmodelY.initRls( rls_lambda, rls_sigma );
	//  outerLoopRLSmodelY.initPos( cartIniY );

	// CT RLS
	outerLoopCTRLSmodelX.updateDelT( outerLoopTime );
	outerLoopCTRLSmodelX.updateAB( task_mA,
			task_mB );

	outerLoopCTRLSmodelY.updateDelT( outerLoopTime );
	outerLoopCTRLSmodelY.updateAB( task_mA,
			task_mB );

	// MSD
	outerLoopMSDmodelX.updateDelT( outerLoopTime );
	outerLoopMSDmodelX.updateMsd( m_M,
			m_S,
			m_D );

	outerLoopMSDmodelY.updateDelT( outerLoopTime );
	outerLoopMSDmodelY.updateMsd( m_M,
			m_S,
			m_D );

	// IRL
	outerLoopIRLmodelX.init( numCartDof, numIrlSamples, numIrlLsIter, irlOneshot );
	outerLoopIRLmodelX.updateDelT( outerLoopTime );
	outerLoopIRLmodelX.updateMsd( m_M,
			m_S,
			m_D );
	outerLoopIRLmodelX.updateAB( task_mA,
			task_mB );

	outerLoopIRLmodelY.init( numCartDof, numIrlSamples, numIrlLsIter, irlOneshot );
	outerLoopIRLmodelY.updateDelT( outerLoopTime );
	outerLoopIRLmodelY.updateMsd( m_M,
			m_S,
			m_D );
	outerLoopIRLmodelY.updateAB( task_mA,
			task_mB );


	/////////////////////////

	// System Model END
	/////////////////////////


	return result;
}

bool PR2CartneuroControllerClass::initInnerLoop()
{
	std::string nn_kappa            = "/nn_kappa"            ;
	std::string nn_Kv               = "/nn_Kv"               ;
	std::string nn_lambda           = "/nn_lambda"           ;
	std::string nn_Kz               = "/nn_Kz"               ;
	std::string nn_Zb               = "/nn_Zb"               ;
	std::string nn_feedForwardForce = "/nn_feedForwardForce" ;
	std::string nn_nnF              = "/nn_nnF"              ;
	std::string nn_nnG              = "/nn_nnG"              ;
	std::string nn_ONparam          = "/nn_ON"               ;

	if (!nh_.getParam( nn_kappa            , kappa            ))
	{ ROS_ERROR("Value not loaded from parameter: %s !)", nn_kappa.c_str())                        ; return false; }
	if (!nh_.getParam( nn_Kv               , Kv               ))
	{ ROS_ERROR("Value not loaded from parameter: %s !)", nn_Kv.c_str())                           ; return false; }
	if (!nh_.getParam( nn_lambda           , lambda           ))
	{ ROS_ERROR("Value not loaded from parameter: %s !)", nn_lambda.c_str())                       ; return false; }
	if (!nh_.getParam( nn_Kz               , Kz               ))
	{ ROS_ERROR("Value not loaded from parameter: %s !)", nn_Kz.c_str())                           ; return false; }
	if (!nh_.getParam( nn_Zb               , Zb               ))
	{ ROS_ERROR("Value not loaded from parameter: %s !)", nn_Zb.c_str())                           ; return false; }
	if (!nh_.getParam( nn_feedForwardForce , fFForce ))
	{ ROS_ERROR("Value not loaded from parameter: %s !)", nn_feedForwardForce.c_str())             ; return false; }
	if (!nh_.getParam( nn_nnF              , nnF              ))
	{ ROS_ERROR("Value not loaded from parameter: %s !)", nn_nnF.c_str())                          ; return false; }
	if (!nh_.getParam( nn_nnG              , nnG              ))
	{ ROS_ERROR("Value not loaded from parameter: %s !)", nn_nnG.c_str())                          ; return false; }
	if (!nh_.getParam( nn_ONparam          , nn_ON            ))
	{ ROS_ERROR("Value not loaded from parameter: %s !)", nn_ONparam.c_str())                      ; return false; }

	std::string para_nnNum_Inputs  = "/nnNum_Inputs" ;
	std::string para_nnNum_Outputs = "/nnNum_Outputs" ;
	std::string para_nnNum_Hidden  = "/nnNum_Hidden" ;
	std::string para_nnNum_Error   = "/nnNum_Error" ;
	std::string para_nnNum_Joints  = "/nnNum_Joints" ;

	if (!nh_.getParam( para_nnNum_Inputs , num_Inputs  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Inputs .c_str()) ; return false; }
	if (!nh_.getParam( para_nnNum_Outputs, num_Outputs )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Outputs.c_str()) ; return false; }
	if (!nh_.getParam( para_nnNum_Hidden , num_Hidden  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Hidden .c_str()) ; return false; }
	if (!nh_.getParam( para_nnNum_Error  , num_Error   )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Error  .c_str()) ; return false; }
	if (!nh_.getParam( para_nnNum_Joints , num_Joints  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Joints .c_str()) ; return false; }

	return true;
}


