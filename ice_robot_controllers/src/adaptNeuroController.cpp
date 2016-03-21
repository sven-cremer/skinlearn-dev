#include "ice_robot_controllers/adaptNeuroController.h"
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS( pr2_controller_ns::PR2adaptNeuroControllerClass, pr2_controller_interface::Controller)

PR2adaptNeuroControllerClass::PR2adaptNeuroControllerClass()
: robot_state_(NULL)			// Initialize variables
{
}

PR2adaptNeuroControllerClass::~PR2adaptNeuroControllerClass()
{
	m_Thread.interrupt();		// Kill thread at one of the interruption points
	for(int i=0; i<numTactileSensors_;i++)
	{
		delete ARMAmodel_flexi_[i];
	}
}

/// Controller initialization in non-realtime
bool PR2adaptNeuroControllerClass::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
	ROS_INFO("Initializing Neuroadpative Controller...");

	// Store node handle
	nh_ = n;
	// Test if we got robot pointer
	assert(robot);
	// Store the robot handle for later use (to get time).
	robot_state_ = robot;

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
	if( !initNullspace() )
	{
		ROS_ERROR("initNullspace() failed!");
		return false;
	}

	// Subscribe to Flexiforce wrench commands
	if(useFlexiForce)
	{
		sub_tactileWrench_ = nh_.subscribe<geometry_msgs::WrenchStamped>("/tactile/wrench", 1, &PR2adaptNeuroControllerClass::readForceValuesCB, this);
		sub_tactileData_   = nh_.subscribe<ice_msgs::tactileArrayData>("/tactile/data", 1, &PR2adaptNeuroControllerClass::readTactileDataCB, this);
	}
	tactileCalibration_srv_ = nh_.advertiseService("/tactile/calibration" , &PR2adaptNeuroControllerClass::tactileCalibrationCB   , this);
	status_srv_ = nh_.advertiseService("/tactile/status" , &PR2adaptNeuroControllerClass::statusCB   , this);
	tactileFilterWeights_srv_ = nh_.advertiseService("/tactile/filterWeights" , &PR2adaptNeuroControllerClass::tactileFilterWeightsCB       , this);



	runExperimentA_srv_ = nh_.advertiseService("runExperimentA" , &PR2adaptNeuroControllerClass::runExperimentA   , this);
	runExperimentB_srv_ = nh_.advertiseService("runExperimentB" , &PR2adaptNeuroControllerClass::runExperimentB   , this);
	runExperimentC_srv_ = nh_.advertiseService("runExperimentC" , &PR2adaptNeuroControllerClass::runExperimentC   , this);

	// NN weights
	updateInnerNNweights_srv_  = nh_.advertiseService("updateInnerNNweights" , &PR2adaptNeuroControllerClass::updateInnerNNweights   , this);
	updateNNweights_srv_  = nh_.advertiseService("updateNNweights" , &PR2adaptNeuroControllerClass::updateNNweights   , this);
	setNNweights_srv_  = nh_.advertiseService("setNNweights" , &PR2adaptNeuroControllerClass::setNNweights   , this);
	getNNweights_srv_  = nh_.advertiseService("getNNweights" , &PR2adaptNeuroControllerClass::getNNweights   , this);

	/////////////////////////
	// DATA COLLECTION
	publish_srv_             = nh_.advertiseService("publishExpData"     , &PR2adaptNeuroControllerClass::publishExperimentData              , this);
	capture_srv_             = nh_.advertiseService("capture"            , &PR2adaptNeuroControllerClass::capture              , this);
	setRefTraj_srv_          = nh_.advertiseService("setRefTraj"         , &PR2adaptNeuroControllerClass::setRefTraj           , this);
	toggleFixedWeights_srv_  = nh_.advertiseService("toggleFixedWeights" , &PR2adaptNeuroControllerClass::toggleFixedWeights   , this);

	pubFTData_               = nh_.advertise< geometry_msgs::WrenchStamped >( "FT_data"              , StoreLen);
	pubModelStates_          = nh_.advertise< sensor_msgs::JointState      >( "model_joint_states"   , StoreLen);
	pubRobotStates_          = nh_.advertise< sensor_msgs::JointState      >( "robot_joint_states"   , StoreLen);
	pubModelCartPos_         = nh_.advertise< geometry_msgs::PoseStamped   >( "model_cart_pos"       , StoreLen);
	pubRobotCartPos_         = nh_.advertise< geometry_msgs::PoseStamped   >( "robot_cart_pos"       , StoreLen);
	pubControllerParam_      = nh_.advertise< ice_msgs::controllerParam    >( "controller_params"    , StoreLen);
	pubControllerFullData_   = nh_.advertise< ice_msgs::controllerFullData >( "controllerFullData"   , StoreLen);
	pubExperimentDataA_	     = nh_.advertise< ice_msgs::experimentDataA    >( "experimentDataA"   , StoreLen);
	pubExperimentDataB_	     = nh_.advertise< ice_msgs::experimentDataB    >( "experimentDataB"   , StoreLen);
	pubExperimentDataC_	     = nh_.advertise< ice_msgs::experimentDataC    >( "experimentDataC"   , StoreLen);

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
void PR2adaptNeuroControllerClass::starting()
{

	// Get the current joint values to compute the initial tip location.
	chain_.getPositions(q0_);
	kin_->fk(q0_, x0_);

	// Initialize the phase of the circle as zero.
	circle_phase = 0.0;
	startCircleTraj = false;

	// Also reset the time-of-last-servo-cycle.
	last_time_     = robot_state_->getTime() ;
	start_time_    = robot_state_->getTime() ;
	outer_elapsed_ = robot_state_->getTime() ;

	// Set starting position
	if( !useCurrentCartPose )
	{
		// Start from specified
		Eigen::Vector3d p_init(cartIniX,cartIniY,cartIniZ);
		Eigen::Quaterniond q_init = euler2Quaternion( cartIniRoll, cartIniPitch, cartIniYaw );
		x0_ = Eigen::Translation3d(p_init) * q_init;
	}
	x_des_ = x0_;
	CartVec p = affine2CartVec(x_des_);
	ROS_INFO("Starting pose: pos=[%f,%f,%f], rot=[%f,%f,%f]",p(0),p(1),p(2),p(3),p(4),p(5));

	// Reference trajectory
	X_m   = affine2CartVec(x_des_);

	// Set transform from accelerometer to FT sensor
	CartVec tmp;
	tmp << 0, 0, 0, 3.142, 1.571, 1.919;	// rosrun tf tf_echo l_force_torque_link l_gripper_motor_accelerometer_link
	x_acc_to_ft_ = CartVec2Affine(tmp);		// TODO: get from robot or load from config file

	// Set filter values
	qdot_filtered_.setZero();

	loop_count_ = 0;
	recordData = false;

	// Start threads
	runComputations = false;
	m_Thread = boost::thread(&PR2adaptNeuroControllerClass::updateNonRealtime, this);

}

/// Parallel thread for updates that are computationally expensive
void PR2adaptNeuroControllerClass::updateNonRealtime()
{
	while(true)
	{
		// 1) Wait
		while(!runComputations)
		{
			boost::this_thread::sleep(boost::posix_time::milliseconds(0.005));
		}

		// 2) Do Computations

		/***************** SENSOR DATA PROCESSING *****************/

		if( forceTorqueOn )		// TODO check if accData has been updated
		{
			// FT compensation

			wrench_gripper_.topRows(3) = gripper_mass * (x_acc_to_ft_.linear()*accData);	// Gripper force

			forceFT =  wrench_gripper_.topRows(3);	// Temporary store values due to Eigen limitations

			wrench_gripper_.bottomRows(3) = r_gripper_com.cross(forceFT); // Torque vector

			wrench_compensated_ = wrench_raw_ - ft_bias - wrench_gripper_;

			forceFT = wrench_compensated_.topRows(3);
			tauFT	= wrench_compensated_.bottomRows(3);

			// Force transformation

			W_mat_(0,0) = 0;
			W_mat_(0,1) = -x_ft_.translation().z();
			W_mat_(0,2) = x_ft_.translation().y();

			W_mat_(1,0) = x_ft_.translation().z();
			W_mat_(1,1) = 0;
			W_mat_(1,2) = -x_ft_.translation().x();

			W_mat_(2,0) = -x_ft_.translation().y();
			W_mat_(2,1) = x_ft_.translation().x();
			W_mat_(2,2) = 0;

			wrench_transformed_.bottomRows(3) = W_mat_*x_ft_.linear()*forceFT + x_ft_.linear()*tauFT;
			wrench_transformed_.topRows(3)    = x_ft_.linear()*forceFT;

			// Apply low-pass filter
			if(useDigitalFilter)		// FIXME wrench_filtered_ not updated if false
			{
		        for(int i=0;i<6;i++)
		        {
		        	wrench_filtered_(i) = digitalFilters[i].getNextFilteredValue(wrench_transformed_(i));
		        }
			}

			transformed_force = wrench_filtered_.topRows(3);
		}

		if(useFlexiForce)
		{
			//tactile_wrench_ = -tactile_wrench_;
			// TODO: update t_r?
			// TODO: transform into torso frame
			transformed_force = tactile_wrench_.topRows(3);		// this variable is being updated by the readForceValuesCB
		}

		// Force threshold (makes force zero bellow threshold) FIXME not needed since force is filtered?
	//	if( ( transformed_force(0) < forceCutOffX ) && ( transformed_force(0) > -forceCutOffX ) ){ transformed_force(0) = 0; }
	//	if( ( transformed_force(1) < forceCutOffY ) && ( transformed_force(1) > -forceCutOffY ) ){ transformed_force(1) = 0; }
	//	if( ( transformed_force(2) < forceCutOffZ ) && ( transformed_force(2) > -forceCutOffZ ) ){ transformed_force(2) = 0; }


		/***************** REFERENCE TRAJECTORY *****************/

		if(executeCircleTraj)
		{
			// Follow a circle at a fixed angular velocity
			circle_phase += circle_rate * dt_;				// w*t = w*(dt1+dt2+dt3+...)

			Eigen::Vector3d p;

			// Set position
			p.x() = cartIniX;
			p.y() = cartIniY + circleAmpl * cos(circle_phase) - circleAmpl;	// Start at cartIniY when circle_phase=0
			p.z() = cartIniZ + circleAmpl * sin(circle_phase);
			x_des_.translation() = p;

			// Set velocity
			p.x() = 0.0;
			p.y() = -circleAmpl * (circle_rate * circleAmpl) * sin(circle_phase);	// TODO CHECK MATH! is phase = A*w*t ???
			p.z() =  circleAmpl * (circle_rate * circleAmpl) * cos(circle_phase);
			xd_des_.translation() = p;

			// Set acceleration
			p.x() = 0.0;
			p.y() = -circleAmpl * (circle_rate * circle_rate * circleAmpl) * cos(circle_phase);
			p.z() = -circleAmpl * (circle_rate * circle_rate * circleAmpl) * sin(circle_phase);
			xdd_des_.translation() = p;

	//		if( x_des_.translation() == x0_.translation() )
	//			loopsCircleTraj++;

			//cartIniX,cartIniY,cartIniZ

			if(circle_phase > (2*3.14159)*numCircleTraj)
			{
				executeCircleTraj = false;
			}
		}
		if(mannequinMode && loop_count_ > 3000) // Check if initialized
		{

			// Compute error
			computePoseError(x_, x_des_, xerr_);
	//		Eigen::Vector3d tmp1; tmp1 << xerr_.(0),xerr_.(1),xerr_.(2);
	//		Eigen::Vector3d tmp2; tmp2 << xerr_.(3),xerr_.(4),xerr_.(5);
	//		if(tmp1.norm() > mannequinThresPos)
	//		{
	//			x_des_ = x_;
	//		}
	//		if(tmp2.norm() > mannequinThresRot)
	//		{
	//			x_des_ = x_;
	//		}
			if(xerr_.norm() > mannequinThresPos)	// TODO: implement two threshold
			{
				x_des_ = x_;
			}
		}
		if(useHumanIntent && loop_count_ > 3000)
		{

//			if( ( robot_state_->getTime() - intent_elapsed_ ).toSec() >= intentLoopTime )
//			{
//				task_ref = x_des_.translation();
//
//				calcHumanIntentPos( transformed_force, task_ref, intentEst_delT, intentEst_M );
//
//				// Transform human intent to torso lift link
//				CartVec xyz = affine2CartVec(x_acc_);
//				task_ref.x() = xyz(1) + task_ref.x() ;
//				task_ref.y() = xyz(2) + task_ref.y() ;
//				task_ref.z() = xyz(3) + task_ref.z() ;

			// TODO check limits?			a = F/m							v=v0+at			x=x0+vt
			x_des_.translation() += (transformed_force / intentEst_M) * intentEst_delT * intentEst_delT;

//
//				x_des_.translation() = task_ref;
//
//				intent_elapsed_ = robot_state_->getTime() ;
//			}
		}
		if(calibrateSensors)	// TODO make x_r step, task model is inside RlsModel
		{
			// Generate x_r with xd_r ~ V
			// prev_x_r = x_r;

			// Calibrate all sensors
			if(!calibrateSingelSensors)
			{
				for(int i=0; i<numTactileSensors_;i++)
				{
					if(externalRefTraj)	// Use a fixed x_r
					{
						if(tactile_data_(i)>0)
							xdd_r.topRows(3) = -(1/intentEst_M)*sensorDirections.col(i);
					}
					else	// Make voltage proportional to user input
					{
						xdd_r.topRows(3) = -(1/intentEst_M)*tactile_data_(i)*sensorDirections.col(i);
					}
					xd_r .topRows(3) += xdd_r.topRows(3)*dt_;
					x_r  .topRows(3) +=  xd_r.topRows(3)*dt_;
				}
			}
			else // Calibration single sensors
			{
				// Wait for first contact before setting x_r
				if(tactile_data_(tactileSensorSelected_)>0 && !refTrajSetForCalibration)
				{
					// Step function
					x_r.topRows(3) = x0_cali_vec_.topRows(3) + maxCalibrationDistance_*sensorDirections.col(tactileSensorSelected_);
					x_d.topRows(3) = x0_cali_vec_.topRows(3);

					X_m = x0_cali_vec_;	// TODO move?
					calibrationCounter = 0;

					refTrajSetForCalibration = true;
					// Start data recording
					experiment_ = PR2adaptNeuroControllerClass::B;
					storage_index_ = 0;
					recordData = true;
				}
				if(refTrajSetForCalibration)
				{
					calibrationDistance_ = ( x0_cali_.translation() - x_.translation() ).norm();
				}
				/*
				double rate = 0.5;
				xdd_r.topRows(3) =
				xd_r .topRows(3) = rate*tactile_data_(tactileSensorSelected_)*sensorDirections.col(tactileSensorSelected_);
				x_r  .topRows(3) = x_r.topRows(3) + xd_r.topRows(3)*dt_;
				delta_x = ( xd_r.topRows(3)*dt_ ).norm();
				*/
			}

//			delta_x = (x_r - prev_x_r).norm();
//			calibrationDistance_ += delta_x;

			if(calibrationDistance_ > maxCalibrationDistance_) // TODO && time > 1 sec
			{
				calibrationCounter++;
			}

			if( calibrationCounter>333)
			{

				calibrateSensors = false;
				refTrajSetForCalibration = false;
//				xd_r.setZero();
//				xdd_r.setZero();
				// Fix filter weights
				for(int i=0; i<numTactileSensors_;i++)
				{
					ARMAmodel_flexi_[i]->setUseFixedWeights(true);
				}
				//ARMAmodel_flexi_[tactileSensorSelected_]->setUseFixedWeights(true);
				recordData = false;
			}
		}



		/***************** UPDATE LOOP VARIABLES *****************/

		JacobianTrans = J_.transpose();			// [6x7]^T->[7x6]

		t_r.setZero();		// [6x1] (num_Outputs)
		tau_.setZero();		// [7x1] (num_Joints)

		// Current joint positions and velocities
		q = q_;
		qd = qdot_;

		X = affine2CartVec(x_);
		Xd = xdot_;				// FIXME make sure they are of same type

//		// Reference trajectory
//		X_m   = affine2CartVec(x_des_);
//		Xd_m  = affine2CartVec(xd_des_);
//		Xdd_m = affine2CartVec(xdd_des_);

		/***************** FEEDFORWARD FORCE *****************/

//		if(useFlexiForce)
//		{
//			tau_ = JacobianTrans*(-fFForce*tactile_wrench_);	// [7x6]*[6x1]->[7x1]
//		}

		if(forceTorqueOn)
		{
			tau_ = JacobianTrans*(fFForce*wrench_filtered_);	// [7x6]*[6x1]->[7x1]
		}

		/***************** OUTER LOOP *****************/

		if(useOuterloop && loop_count_ > 3000)
		{
			// Variables previously updated:
			//	Xd_m				-> Model reference trajectory velocity
			//	Xd					-> Actual velocity
			//	X_m					-> Model reference trajectory position
			//	X					-> Actual position
			//	Xdd_m				-> Model reference trajectory acceleration
			//	transformed_force	-> from flexiforce or FT sensor
			//	task_ref			-> Reference trajectory from human intent

			//updateOuterLoop();

			outer_delT = (robot_state_->getTime() - outer_elapsed_ ).toSec();

//			task_ref.topRows(3) = x_des_.translation();
//			X_m.setZero();
//			Xd_m.setZero();
//			Xdd_m.setZero();
//			task_refModel_output.setZero();

			// Transform x_di
//			sensor_0 : [1,0]
//			sensor_1 : [0,-1]
//			sensor_2 : [-1,0]
//			sensor_3 : [0,1]

//			X_m   .setZero();
//			Xd_m  .setZero();
//			Xdd_m .setZero();

			if(calibrateSensors  && refTrajSetForCalibration)	// TODO: check if tactileSensorSelected_ within range
			{
				Eigen::VectorXd tmp;
				tmp.resize(4);
				tmp.setZero();

				// Compute x_d
				prev_x_d = x_d;
				x_d.topRows(3) = ( task_mA*outer_delT*x_r.topRows(3) + prev_x_d.topRows(3) ) / ( 1+task_mB*outer_delT );

				// Project into sensor axis to get -x_d
				tmp(0) = x_d.topRows(3).dot( sensorDirections.col(tactileSensorSelected_) );

				// Update ARMA model and get x_m
				ARMAmodel_flexi_[tactileSensorSelected_]->runARMAupdate(outer_delT  ,  // input: delta T
							tactile_data_(tactileSensorSelected_)        			,  // input:  force or voltage
							tmp(0)      											,  // input:  x_d
							tmp(1)      											,  // output: x_m
							tmp(2)     												,  // output: xd_m
							tmp(3)    												);  // output: xdd_m
//std::cout<<"ARMA:\n"<<tmp<<"\n---\n";
				// Convert into global reference frame

				if(tmp(2)>1)
					tmp(2)=1;
				if(tmp(2)<1)
					tmp(2)=-1;

				X_m.topRows(3) = x0_cali_vec_.topRows(3).cwiseProduct(Vec3d_ones - sensorDirections.col(tactileSensorSelected_).cwiseAbs());
				X_m.topRows(3) += tmp(1)*sensorDirections.col(tactileSensorSelected_);

				Xd_m .topRows(3) = tmp(2)*sensorDirections.col(tactileSensorSelected_); // or tmp(0)/delT   ?
//				Xdd_m.topRows(3) += tmp(3)*sensorDirections.col(tactileSensorSelected_); // or tmp(0)/delT^2 ?		// FIXME
//std::cout<<"X_m:\n"<<X_m<<"\n---\n";
				// Save weights
				for(int i=0; i<numTactileSensors_;i++)
				{
					Eigen::MatrixXd tmp2;
					ARMAmodel_flexi_[i]->getWeights(tmp2);
					filterWeights_flexi_.col(i) = tmp2;
				}

			}
			else if(tactileSensorSelected_==-1)
			{
//				for(int i=0;i<numTactileSensors_;i++)
//				{
//					if(tactile_data_(i)>0)
//					{
//						X_m   .setZero();		// Issue: should not be done before calibration
//						Xd_m  .setZero();
//						Xdd_m .setZero();
//					}
//				}

				Eigen::Vector3d tmp;
				for(int i=0;i<numTactileSensors_;i++)
				{
					ARMAmodel_flexi_[i]->updateDelT(outer_delT);
					ARMAmodel_flexi_[i]->useARMA( tmp(0),				// output: x_m
												  tmp(1),				// output: xd_m
												  tmp(2),				// output: xdd_m
												  tactile_data_(i) );	// input:  force or voltage

					X_m  .topRows(3) += tmp(0)*sensorDirections.col(i);
					Xd_m .topRows(3) += tmp(1)*sensorDirections.col(i);
					Xdd_m.topRows(3) += tmp(2)*sensorDirections.col(i);
				}

				// Save weights
//				for(int i=0; i<numTactileSensors_;i++)
//				{
//					Eigen::MatrixXd tmp2;
//					ARMAmodel_flexi_[i]->getWeights(tmp2);
//					filterWeights_flexi_.col(i) = tmp2;
//				}
			}

			outer_elapsed_ = robot_state_->getTime() ;

			// Output variables:
			//   x_d;
			//   X_m;
			//   Xd_m;
			//   Xdd_m;

		}

		/***************** INNER LOOP *****************/

		// Neural Network

		nnController.updateDelT( dt_ );

		nnController.UpdateCart( X     ,
	                             Xd    ,
	                             X_m   ,
	                             Xd_m  ,
	                             Xdd_m ,
	                             q     ,
	                             qd    ,
	                             t_r   ,			// Feedforward force [6x1]
	                             force_c  );		// Output [6x1]


/* Experiment C
 * RBF and TANH Neural Network

		tau.setZero();

		if(experiment_ == PR2adaptNeuroControllerClass::C && recordData)
		{
			circle_phase += circle_rate * dt_;				// w*t = w*(dt1+dt2+dt3+...)

			q_m(3) 		=  -1.0 + circleAmpl * cos(circle_phase) - circleAmpl/2;
			qd_m(3)		= -circleAmpl * circle_rate * sin(circle_phase);
			qdd_m(3)	= -circleAmpl * circle_rate * circle_rate * cos(circle_phase);

			rbfnnController.update( dt_		,		// time step
									q		,		// x1
									qd		,		// x2
									q_m		,		// xd
									qd_m	,		// vd
									qdd_m	,		// ad
									tau  );
			tau(0) *= 0.05;
			tau(1) *= 0.05;
			tau(2) *= 0.05; // FIXME temporary solution
			tau(4) *= 0.05;
			tau(5) *= 0.05;
			tau(6) *= 0.05;
		}
		if(experiment_ == PR2adaptNeuroControllerClass::C && !recordData)
		{
			q_m.setZero();
			q_m(3) = -1.0 + circleAmpl/2;;
			tau = 50.0*(q_m-q);
		}

		tau_ = tau;
*/

		// PD controller
/*
		// Calculate a Cartesian restoring force.
		computePoseError(x_, x_des_, xerr_);			// TODO: Use xd_filtered_ instead

		CartVec kp, kd;
		kp << 100.0,100.0,100.0,100.0,100.0,100.0;
		kd << 1.0,1.0,1.0,1.0,1.0,1.0;
		// F    = -(       Kp * (x-x_dis)   +     Kd * (xdot - 0)    )
		force_c = -(kp.asDiagonal() * xerr_ + kd.asDiagonal() * xdot_);			// TODO choose NN/PD with a param
*/

		tau_ = tau_ + JacobianTrans*force_c;		// [7x6]*[6x1]->[7x1]

		/***************** NULLSPACE *****************/

		// Computes the desired joint torques for achieving the posture
		if (useNullspacePose)
		{
			nullspaceTorque.setZero();

			// Computes pseudo-inverse of J
			JJt_damped = J_ * JacobianTrans + jacobian_inverse_damping * IdentityCart;
			JJt_inv_damped = JJt_damped.inverse();
			J_pinv = JacobianTrans * JJt_inv_damped;

			// Computes the nullspace of J
			nullSpace = IdentityJoints - J_pinv * J_;

			for (int j = 0; j < Joints; ++j)
			{
				// This is the Liegeois cost function from 1977
				q_jointLimit(j) = - (q(j) - qnom(j) )/( q.rows() * ( q_upper(j) - q_lower(j))) ;
			}
			nullspaceTorque = nullSpace*50*( q_jointLimit - 0.0*qd );

			tau_ = tau_ + nullspaceTorque;
		}

		/***************** TORQUE *****************/

		// Torque Saturation							(if a torque command is larger than the saturation value, then scale all torque values such that torque_i=saturation_i)
		for (int i = 0; i < num_Joints; ++i)
		{
			if (saturation_[i] > 0.0)
				sat_scaling = std::min(sat_scaling, fabs(saturation_[i] / tau_[i]));
		}
		tau_sat = sat_scaling * tau_;

		tau_c_ = JointEigen2Kdl( tau_sat );

		runComputations = false;

	}
}


/// Controller update loop in realtime
void PR2adaptNeuroControllerClass::update()
{

	++loop_count_;	// Start at 1

	if(loop_count_ % loopRateFactor == 1 && !runComputations)	// Retrieve data and start computations in a thread
	{
		// Calculate the dt between servo cycles.
		dt_ = (robot_state_->getTime() - last_time_).toSec();
		last_time_ = robot_state_->getTime();

		// Get the current joint positions and velocities.
		chain_.getPositions(q_);
		chain_.getVelocities(qdot_);

		// Get the pose of the F/T sensor
		if(forceTorqueOn)
		{
			kin_ft_->fk(q_,x_ft_);
			//kin_ft_->jac(q_,J_ft_);
			//kin_acc_to_ft_->fk(q_,x_acc_to_ft_);
		}

		// Compute the forward kinematics and Jacobian (at this location).
		kin_->fk(q_, x_);
		kin_->jac(q_, J_);		// [6x7]

		// Get accelerometer forward kinematics and Jacobian
		//	kin_acc_->fk(q_, x_acc_);				TODO update value for outloop
		//	kin_acc_->jac(q_, J_acc_);

		// TODO Filter velocity values with 2nd order butterworth
		/*
		chain_.getVelocities(qdot_raw_);
		for (int i = 0; i < Joints; ++i)
			qdot_filtered_[i] += joint_vel_filter_ * (qdot_raw_[i] - qdot_filtered_[i]);	// Does nothing when joint_vel_filter_=1
		qdot_ = qdot_filtered_;
		*/

		xdot_ = J_ * qdot_;		// [6x7]*[7x1] -> [6x1]


		// Estimate force at gripper tip
		//	chain_.getEfforts(tau_measured_);
		//	for (unsigned int i = 0 ; i < 6 ; i++)
		//	{
		//		force_measured_(i) = 0;
		//		for (unsigned int j = 0 ; j < Joints ; j++)
		//			force_measured_(i) += J_(i,j) * tau_measured_(j);
		//	}

		transformed_force.setZero();

		/***************** GET SENSOR DATA *****************/

		// Flexiforce data updated by readForceValuesCB
		if(useFlexiForce)
		{
			ros::spinOnce();	// TODO move to nonRealtime update, seems to slow down loop
		}

		if( accelerometerOn )//|| forceTorqueOn )
		{
			// Retrieve accelerometer data
			accData_vector.clear();
			accData_vector = accelerometer_handle_->state_.samples_;
			accData_received = true;

			// Convert into Eigen vector and compute average value
			accData.setZero();
			accData_vector_size = accData_vector.size();		// 3 or 4 (usually three)
			for( int  i = 0; i < accData_vector_size; i++ )		// Take average value
			{
				accData(0) += accData_vector[i].x;
				accData(1) += accData_vector[i].y;
				accData(2) += accData_vector[i].z;
			}
			accData = accData.array() / (double)accData_vector_size;
		}

		if( forceTorqueOn )
		{
			// Retrieve Force/Torque data
			ftData_vector.clear();
			ftData_vector = ft_handle_->state_.samples_;
			ftData_received = true;

			// Convert into Eigen vector
			ftData_vector_size = ftData_vector.size();					// 2,3,4 (usually three)
			ftData_msg.wrench = ftData_vector[ftData_vector_size-1];	// Take latest value
			tf::wrenchMsgToEigen(ftData_msg.wrench, wrench_raw_);
		}


		/***************** START DATA PROCESSING *****************/
		runComputations = true;

	}

	if(loop_count_ % loopRateFactor == 0)	// After X loops, assume computations are done and send commands
	{

		if(runComputations)
		{
			cartvec_tmp(0)++;	// computation took too long
			return;
		}
		else
		{
			tau_c_latest_ = tau_c_;
		}

		/***************** DATA COLLECTION *****************/

		if (recordData)
		{
			bufferData();
		}

	}

	// Send torque command
	if(loop_count_ > loopRateFactor)
	{
		chain_.setEfforts( tau_c_latest_ );
	}

	/***************** DATA PUBLISHING *****************/

	if (publishRTtopics && loop_count_ % loopRateFactor == 0 )
	{
		//cartvec_tmp(1) = accData_vector_size;
		//cartvec_tmp(2) = ftData_vector_size;
		//cartvec_tmp(1) = nnController.getOuterWeightsNorm();
		//cartvec_tmp(2) = nnController.getInnerWeightsNorm();
		cartvec_tmp(1) = calibrationDistance_;
		cartvec_tmp(2) = filterWeights_flexi_.norm();

		if (pub_x_desi_.trylock()) {
			pub_x_desi_.msg_.header.stamp = last_time_;
			tf::poseEigenToMsg(CartVec2Affine(cartvec_tmp), pub_x_desi_.msg_.pose);	// cartvec_tmp
			//tf::poseEigenToMsg(x_acc_to_ft_, pub_x_desi_.msg_.pose);
			pub_x_desi_.msg_.header.frame_id = "l_gripper_motor_accelerometer_link";
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


		if (pub_ft_.trylock()) {
			pub_ft_.msg_.header.stamp = last_time_;
			tf::wrenchEigenToMsg(wrench_compensated_, pub_ft_.msg_.wrench);
			//pub_ft_.msg_.wrench = l_ftData.wrench;
			pub_ft_.unlockAndPublish();
		}

		if (pub_ft_transformed_.trylock()) {
			pub_ft_transformed_.msg_.header.stamp = last_time_;
			tf::wrenchEigenToMsg(wrench_filtered_, pub_ft_transformed_.msg_.wrench);
			pub_ft_transformed_.unlockAndPublish();
		}
*/
	}



}

void PR2adaptNeuroControllerClass::updateOuterLoop()
{
	// OUTER Loop Update

	// Human Intent Estimation
	/*
	if( !externalRefTraj )
	{
		if( ( robot_state_->getTime() - intent_elapsed_ ).toSec() >= intentLoopTime )
		{
			calcHumanIntentPos( transformed_force, task_ref, intentEst_delT, intentEst_M );

			// Transform human intent to torso lift link
			CartVec xyz = affine2CartVec(x_acc_);
			task_ref.x() = xyz(1) + task_ref.x() ;
			task_ref.y() = xyz(2) + task_ref.y() ;
			task_ref.z() = xyz(3) + task_ref.z() ;

			intent_elapsed_ = robot_state_->getTime() ;
		}
	}
	*/

	outer_delT = (robot_state_->getTime() - outer_elapsed_ ).toSec();

	if( outer_delT >= outerLoopTime )
	{
		// RLS ARMA
		if( useARMAmodel )
		{
/*
			if( useFlexiForce )	// One model per axis containing all the sensors
			{
				// Set ARMA parameters
				// Note: tactile_wrench_ corresponds to the applied force (not the reactant force)

				// X axis
				if( tactile_wrench_(0) > 0.0 )	// Positive if pressing 0 (i.e. forward)
				{

					outerLoopRLSmodelX.setWeights( outerLoopWk_flexi_0 ) ;
					if( useFixedWeights )
						outerLoopRLSmodelX.setFixedWeights( outerLoopWk_flexi_0 );
					else
						outerLoopRLSmodelX.setUpdatedWeights();

				}
				else							// Negative if pressing 2 (i.e. backward)
				{
					outerLoopRLSmodelX.setWeights( outerLoopWk_flexi_2 ) ;
					if( useFixedWeights )
						outerLoopRLSmodelX.setFixedWeights( outerLoopWk_flexi_2 );
					else
						outerLoopRLSmodelX.setUpdatedWeights();
				}

				// Y axis
				if( tactile_wrench_(1) > 0.0 )	// Positive if pressing 1 (i.e. outward)
				{
					outerLoopRLSmodelY.setWeights( outerLoopWk_flexi_1 ) ;
					if( useFixedWeights )
						outerLoopRLSmodelY.setFixedWeights( outerLoopWk_flexi_1 );
					else
						outerLoopRLSmodelY.setUpdatedWeights();
				}
				else							// Negative if pressing 3 (i.e. inward)
				{
					outerLoopRLSmodelY.setWeights( outerLoopWk_flexi_3 ) ;
					if( useFixedWeights )
						outerLoopRLSmodelY.setFixedWeights( outerLoopWk_flexi_3 );
					else
						outerLoopRLSmodelY.setUpdatedWeights();
				}

			}


			// Raw tactile data (voltages)

			if(0 <= tactileSensorSelected_ && tactileSensorSelected_ < numTactileSensors_)	// Calibration mode		FIXME: use useFixedWeights instead
			{
				//ARMAmodel_flexi_[tactileSensorSelected_].setUpdatedWeights();
				for(int i=0; i<numTactileSensors_*tactile_dimensions_;i++)
				{
					ARMAmodel_flexi_[i]->setUpdatedWeights();
				}
			}
			else							// Fixed weights
			{
				for(int i=0; i<numTactileSensors_*tactile_dimensions_;i++)
				{
					Eigen::MatrixXd tmp = filterWeights_flexi_.col(i);
					ARMAmodel_flexi_[i]->setFixedWeights(tmp);
				}
			}
*/
			task_ref.topRows(3) = x_des_.translation();

			X_m.setZero();
			Xd_m.setZero();
			Xdd_m.setZero();
			task_refModel_output.setZero();

			int sensorNr = 0;
			CartVec tmp;
			for(int d=0; d<tactile_dimensions_;d++)
			{
				for(int i=0;i<numTactileSensors_;i++)
				{
					int k = i + numTactileSensors_*d;	// Sensor number

					ARMAmodel_flexi_[k]->updateDelT(outer_delT);
					ARMAmodel_flexi_[k]->updateARMA(tmp                    (1) ,   // output: xd_m
												    Xd                     (0) ,
													tmp                    (0) ,   // output: x_m
												    X                      (0) ,
													tmp                    (3) ,   // output: xdd_m
												    tactile_data_      	   (i) ,   // input:  force or voltage
												    task_ref               (d) ,   // input:  x_r
													tmp                    (4)  ); // output: x_d

					X_m(d) 					+= tmp(0);
					Xd_m(d) 				+= tmp(1);
					Xdd_m(d) 				+= tmp(2);
					task_refModel_output(d)	+= tmp(4);
				}
			}

			for(int i=0; i<numTactileSensors_*tactile_dimensions_;i++)
			{
				Eigen::MatrixXd tmp2;
				ARMAmodel_flexi_[i]->getWeights(tmp2);
				filterWeights_flexi_.col(i) = tmp2;
			}


/*
			// X axis (use 1D update)
			outerLoopRLSmodelX.updateDelT(current_delT);
			outerLoopRLSmodelX.updateARMA( Xd_m                   (0) ,   // output: xd_m
                                           Xd                     (0) ,
                                           X_m                    (0) ,   // output: x_m
                                           X                      (0) ,
                                           Xdd_m                  (0) ,   // output: xdd_m
                                           transformed_force      (0) ,   // input:  force or voltage
                                           task_ref               (0) ,   // input:  x_r
                                           task_refModel_output   (0)  ); // output: x_d

			// Y axis
			outerLoopRLSmodelY.updateDelT(current_delT);
			outerLoopRLSmodelY.updateARMA( Xd_m                   (1) ,
                                           Xd                     (1) ,
                                           X_m                    (1) ,
                                           X                      (1) ,
                                           Xdd_m                  (1) ,
                                           transformed_force      (1) ,
                                           task_ref               (1) ,
                                           task_refModel_output   (1)  );

			outerLoopRLSmodelX.getWeights( weightsRLSmodelX ) ;
			outerLoopRLSmodelY.getWeights( weightsRLSmodelY ) ;

			// ROS_ERROR_STREAM("USING RLS ARMA");

			if( useFlexiForce )
			{
				// X axis
				if( tactile_wrench_(0) > 0.0 )
				{
					outerLoopRLSmodelX.getWeights( outerLoopWk_flexi_0 ) ;
				}
				else
				{
					outerLoopRLSmodelX.getWeights( outerLoopWk_flexi_2 ) ;
				}

				// Y axis
				if( tactile_wrench_(1) > 0.0 )
				{
					outerLoopRLSmodelY.getWeights( outerLoopWk_flexi_1 ) ;
				}
				else
				{
					outerLoopRLSmodelY.getWeights( outerLoopWk_flexi_3 ) ;
				}

			}
			else
			{
				outerLoopRLSmodelX.getWeights( outerLoopWk ) ;
				outerLoopRLSmodelY.getWeights( outerLoopWk ) ;
			}
*/
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

void PR2adaptNeuroControllerClass::bufferData()
{
	if( (storage_index_ < 0) || (storage_index_ >= StoreLen) )
	{
		recordData = false;
		return;
	}


	if (experiment_ == PR2adaptNeuroControllerClass::A)
	{
		//                tf::PoseKDLToMsg(x_m_, modelCartPos_);
		//                tf::PoseKDLToMsg(x_  , robotCartPos_);

		experimentDataA_msg_[storage_index_].dt                = dt_;

		// Neural network
//		tf::matrixEigenToMsg(nnController.getInnerWeights(),experimentDataA_msg_[index].net.V);
//		tf::matrixEigenToMsg(nnController.getOuterWeights(),experimentDataA_msg_[index].net.W);
//	    experimentDataA_msg_[index].net.num_Inputs = num_Inputs;
//		experimentDataA_msg_[index].net.num_Hidden = num_Hidden;
//		experimentDataA_msg_[index].net.num_Outputs = num_Outputs;
		experimentDataA_msg_[storage_index_].Wnorm = nnController.getOuterWeightsNorm();
		experimentDataA_msg_[storage_index_].Vnorm = nnController.getInnerWeightsNorm();

		// Actual trajectory
		experimentDataA_msg_[storage_index_].x_x     = X(0);
		experimentDataA_msg_[storage_index_].x_y     = X(1);
		experimentDataA_msg_[storage_index_].x_z     = X(2);
		experimentDataA_msg_[storage_index_].x_phi   = X(3);
		experimentDataA_msg_[storage_index_].x_theta = X(4);
		experimentDataA_msg_[storage_index_].x_psi   = X(5);

		experimentDataA_msg_[storage_index_].xdot_x     = Xd(0);
		experimentDataA_msg_[storage_index_].xdot_y     = Xd(1);
		experimentDataA_msg_[storage_index_].xdot_z     = Xd(2);
		experimentDataA_msg_[storage_index_].xdot_phi   = Xd(3);
		experimentDataA_msg_[storage_index_].xdot_theta = Xd(4);
		experimentDataA_msg_[storage_index_].xdot_psi   = Xd(5);

		// Desired trajectory
		experimentDataA_msg_[storage_index_].xdes_x     = X_m(0);
		experimentDataA_msg_[storage_index_].xdes_y     = X_m(1);
		experimentDataA_msg_[storage_index_].xdes_z     = X_m(2);
		experimentDataA_msg_[storage_index_].xdes_phi   = X_m(3);
		experimentDataA_msg_[storage_index_].xdes_theta = X_m(4);
		experimentDataA_msg_[storage_index_].xdes_psi   = X_m(5);

		// Increment for the next cycle.
		storage_index_ = storage_index_+1;
	}
	else if (experiment_ == PR2adaptNeuroControllerClass::B)
	{
		experimentDataB_msg_[storage_index_].dt                = dt_;
		experimentDataB_msg_[storage_index_].outer_dt          = outer_delT;

		// Neural network
		experimentDataB_msg_[storage_index_].Wnorm = nnController.getOuterWeightsNorm();
		experimentDataB_msg_[storage_index_].Vnorm = nnController.getInnerWeightsNorm();

		// Filter weights
//		tf::matrixEigenToMsg(weightsRLSmodelX, experimentDataB_msg_[storage_index_].filterWeightsX);
//		tf::matrixEigenToMsg(weightsRLSmodelY, experimentDataB_msg_[storage_index_].filterWeightsY);

		experimentDataB_msg_[storage_index_].flexi0.f0 = filterWeights_flexi_(0,0);
		experimentDataB_msg_[storage_index_].flexi0.f1 = filterWeights_flexi_(1,0);
		experimentDataB_msg_[storage_index_].flexi0.f2 = filterWeights_flexi_(2,0);
		experimentDataB_msg_[storage_index_].flexi0.f3 = filterWeights_flexi_(3,0);
		experimentDataB_msg_[storage_index_].flexi0.f4 = filterWeights_flexi_(4,0);
		experimentDataB_msg_[storage_index_].flexi0.f5 = filterWeights_flexi_(5,0);
		experimentDataB_msg_[storage_index_].flexi0.f6 = filterWeights_flexi_(6,0);
		experimentDataB_msg_[storage_index_].flexi0.f7 = filterWeights_flexi_(7,0);

		experimentDataB_msg_[storage_index_].flexi1.f0 = filterWeights_flexi_(0,1);
		experimentDataB_msg_[storage_index_].flexi1.f1 = filterWeights_flexi_(1,1);
		experimentDataB_msg_[storage_index_].flexi1.f2 = filterWeights_flexi_(2,1);
		experimentDataB_msg_[storage_index_].flexi1.f3 = filterWeights_flexi_(3,1);
		experimentDataB_msg_[storage_index_].flexi1.f4 = filterWeights_flexi_(4,1);
		experimentDataB_msg_[storage_index_].flexi1.f5 = filterWeights_flexi_(5,1);
		experimentDataB_msg_[storage_index_].flexi1.f6 = filterWeights_flexi_(6,1);
		experimentDataB_msg_[storage_index_].flexi1.f7 = filterWeights_flexi_(7,1);

		experimentDataB_msg_[storage_index_].flexi2.f0 = filterWeights_flexi_(0,2);
		experimentDataB_msg_[storage_index_].flexi2.f1 = filterWeights_flexi_(1,2);
		experimentDataB_msg_[storage_index_].flexi2.f2 = filterWeights_flexi_(2,2);
		experimentDataB_msg_[storage_index_].flexi2.f3 = filterWeights_flexi_(3,2);
		experimentDataB_msg_[storage_index_].flexi2.f4 = filterWeights_flexi_(4,2);
		experimentDataB_msg_[storage_index_].flexi2.f5 = filterWeights_flexi_(5,2);
		experimentDataB_msg_[storage_index_].flexi2.f6 = filterWeights_flexi_(6,2);
		experimentDataB_msg_[storage_index_].flexi2.f7 = filterWeights_flexi_(7,2);

		experimentDataB_msg_[storage_index_].flexi3.f0 = filterWeights_flexi_(0,3);
		experimentDataB_msg_[storage_index_].flexi3.f1 = filterWeights_flexi_(1,3);
		experimentDataB_msg_[storage_index_].flexi3.f2 = filterWeights_flexi_(2,3);
		experimentDataB_msg_[storage_index_].flexi3.f3 = filterWeights_flexi_(3,3);
		experimentDataB_msg_[storage_index_].flexi3.f4 = filterWeights_flexi_(4,3);
		experimentDataB_msg_[storage_index_].flexi3.f5 = filterWeights_flexi_(5,3);
		experimentDataB_msg_[storage_index_].flexi3.f6 = filterWeights_flexi_(6,3);
		experimentDataB_msg_[storage_index_].flexi3.f7 = filterWeights_flexi_(7,3);

		experimentDataB_msg_[storage_index_].filterNormX = filterWeights_flexi_.norm();
		//experimentDataB_msg_[storage_index_].filterNormY = weightsRLSmodelY.norm();

		// Force input
//		experimentDataB_msg_[storage_index_].tactile_force.x = transformed_force(0);
//		experimentDataB_msg_[storage_index_].tactile_force.y = transformed_force(1);
//		experimentDataB_msg_[storage_index_].tactile_force.z = transformed_force(2);
		experimentDataB_msg_[storage_index_].tactile_data_0  = tactile_data_(0);
		experimentDataB_msg_[storage_index_].tactile_data_1  = tactile_data_(1);
		experimentDataB_msg_[storage_index_].tactile_data_2  = tactile_data_(2);
		experimentDataB_msg_[storage_index_].tactile_data_3  = tactile_data_(3);

		// Actual trajectory
		experimentDataB_msg_[storage_index_].x.x     = X(0);
		experimentDataB_msg_[storage_index_].x.y     = X(1);
		experimentDataB_msg_[storage_index_].x.z     = X(2);
		experimentDataB_msg_[storage_index_].x.phi   = X(3);
		experimentDataB_msg_[storage_index_].x.the   = X(4);
		experimentDataB_msg_[storage_index_].x.psi   = X(5);

		experimentDataB_msg_[storage_index_].xd.x    = Xd(0);
		experimentDataB_msg_[storage_index_].xd.y    = Xd(1);
		experimentDataB_msg_[storage_index_].xd.z    = Xd(2);
		experimentDataB_msg_[storage_index_].xd.phi  = Xd(3);
		experimentDataB_msg_[storage_index_].xd.the  = Xd(4);
		experimentDataB_msg_[storage_index_].xd.psi  = Xd(5);

		// Human intent
		experimentDataB_msg_[storage_index_].x_i.x     = x_r(0);
		experimentDataB_msg_[storage_index_].x_i.y     = x_r(1);
		experimentDataB_msg_[storage_index_].x_i.z     = x_r(2);
		experimentDataB_msg_[storage_index_].x_i.phi   = x_r(3);
		experimentDataB_msg_[storage_index_].x_i.the   = x_r(4);
		experimentDataB_msg_[storage_index_].x_i.psi   = x_r(5);

		// Task trajectory
		experimentDataB_msg_[storage_index_].x_d.x     = x_d(0);
		experimentDataB_msg_[storage_index_].x_d.y     = x_d(1);
		experimentDataB_msg_[storage_index_].x_d.z     = x_d(2);  // = 0
		experimentDataB_msg_[storage_index_].x_d.phi   = x_d(3);  // = 0
		experimentDataB_msg_[storage_index_].x_d.the   = x_d(4);  // = 0
		experimentDataB_msg_[storage_index_].x_d.psi   = x_d(5);  // = 0

		// Model trajectory
		experimentDataB_msg_[storage_index_].x_m.x     = X_m(0);
		experimentDataB_msg_[storage_index_].x_m.y     = X_m(1);
		experimentDataB_msg_[storage_index_].x_m.z     = X_m(2);
		experimentDataB_msg_[storage_index_].x_m.phi   = X_m(3);
		experimentDataB_msg_[storage_index_].x_m.the   = X_m(4);
		experimentDataB_msg_[storage_index_].x_m.psi   = X_m(5);

		experimentDataB_msg_[storage_index_].xd_m.x     = Xd_m(0);
		experimentDataB_msg_[storage_index_].xd_m.y     = Xd_m(1);
		experimentDataB_msg_[storage_index_].xd_m.z     = Xd_m(2);
		experimentDataB_msg_[storage_index_].xd_m.phi   = Xd_m(3);
		experimentDataB_msg_[storage_index_].xd_m.the   = Xd_m(4);
		experimentDataB_msg_[storage_index_].xd_m.psi   = Xd_m(5);

		experimentDataB_msg_[storage_index_].xdd_m.x     = Xdd_m(0);
		experimentDataB_msg_[storage_index_].xdd_m.y     = Xdd_m(1);
		experimentDataB_msg_[storage_index_].xdd_m.z     = Xdd_m(2);
		experimentDataB_msg_[storage_index_].xdd_m.phi   = Xdd_m(3);
		experimentDataB_msg_[storage_index_].xdd_m.the   = Xdd_m(4);
		experimentDataB_msg_[storage_index_].xdd_m.psi   = Xdd_m(5);

	}
	else if (experiment_ == PR2adaptNeuroControllerClass::C)
	{
		// Time
		experimentDataC_msg_[storage_index_].dt                = dt_;
		//experimentDataC_msg_[storage_index_].outer_dt          = outer_delT;

		// NN norms
		experimentDataC_msg_[storage_index_].RBFnorm           = rbfnnController.getRBFNorm();
		experimentDataC_msg_[storage_index_].TANHnorm          = rbfnnController.getTANHNorm();

		// Actual trajectory
		experimentDataC_msg_[storage_index_].q.j0   = q(0);
		experimentDataC_msg_[storage_index_].q.j1   = q(1);
		experimentDataC_msg_[storage_index_].q.j2   = q(2);
		experimentDataC_msg_[storage_index_].q.j3   = q(3);
		experimentDataC_msg_[storage_index_].q.j4   = q(4);
		experimentDataC_msg_[storage_index_].q.j5   = q(5);
		experimentDataC_msg_[storage_index_].q.j6   = q(6);

		experimentDataC_msg_[storage_index_].qd.j0   = qd(0);
		experimentDataC_msg_[storage_index_].qd.j1   = qd(1);
		experimentDataC_msg_[storage_index_].qd.j2   = qd(2);
		experimentDataC_msg_[storage_index_].qd.j3   = qd(3);
		experimentDataC_msg_[storage_index_].qd.j4   = qd(4);
		experimentDataC_msg_[storage_index_].qd.j5   = qd(5);
		experimentDataC_msg_[storage_index_].qd.j6   = qd(6);


		// Reference trajectory
		experimentDataC_msg_[storage_index_].q_d.j0   = q_m(0);
		experimentDataC_msg_[storage_index_].q_d.j1   = q_m(1);
		experimentDataC_msg_[storage_index_].q_d.j2   = q_m(2);
		experimentDataC_msg_[storage_index_].q_d.j3   = q_m(3);
		experimentDataC_msg_[storage_index_].q_d.j4   = q_m(4);
		experimentDataC_msg_[storage_index_].q_d.j5   = q_m(5);
		experimentDataC_msg_[storage_index_].q_d.j6   = q_m(6);

		experimentDataC_msg_[storage_index_].qd_d.j0   = qd_m(0);
		experimentDataC_msg_[storage_index_].qd_d.j1   = qd_m(1);
		experimentDataC_msg_[storage_index_].qd_d.j2   = qd_m(2);
		experimentDataC_msg_[storage_index_].qd_d.j3   = qd_m(3);
		experimentDataC_msg_[storage_index_].qd_d.j4   = qd_m(4);
		experimentDataC_msg_[storage_index_].qd_d.j5   = qd_m(5);
		experimentDataC_msg_[storage_index_].qd_d.j6   = qd_m(6);

		experimentDataC_msg_[storage_index_].qdd_d.j0   = qdd_m(0);
		experimentDataC_msg_[storage_index_].qdd_d.j1   = qdd_m(1);
		experimentDataC_msg_[storage_index_].qdd_d.j2   = qdd_m(2);
		experimentDataC_msg_[storage_index_].qdd_d.j3   = qdd_m(3);
		experimentDataC_msg_[storage_index_].qdd_d.j4   = qdd_m(4);
		experimentDataC_msg_[storage_index_].qdd_d.j5   = qdd_m(5);
		experimentDataC_msg_[storage_index_].qdd_d.j6   = qdd_m(6);

		// Torque
		experimentDataC_msg_[storage_index_].tau.j0   = tau(0);
		experimentDataC_msg_[storage_index_].tau.j1   = tau(1);
		experimentDataC_msg_[storage_index_].tau.j2   = tau(2);
		experimentDataC_msg_[storage_index_].tau.j3   = tau(3);
		experimentDataC_msg_[storage_index_].tau.j4   = tau(4);
		experimentDataC_msg_[storage_index_].tau.j5   = tau(5);
		experimentDataC_msg_[storage_index_].tau.j6   = tau(6);

		experimentDataC_msg_[storage_index_].tau_sat.j0   = tau_sat(0);
		experimentDataC_msg_[storage_index_].tau_sat.j1   = tau_sat(1);
		experimentDataC_msg_[storage_index_].tau_sat.j2   = tau_sat(2);
		experimentDataC_msg_[storage_index_].tau_sat.j3   = tau_sat(3);
		experimentDataC_msg_[storage_index_].tau_sat.j4   = tau_sat(4);
		experimentDataC_msg_[storage_index_].tau_sat.j5   = tau_sat(5);
		experimentDataC_msg_[storage_index_].tau_sat.j6   = tau_sat(6);
	}
	else
	{
		// TODO failure message
	}

	// Increment for the next cycle.
	storage_index_ = storage_index_+1;
}


/// Service call to set reference trajectory
bool PR2adaptNeuroControllerClass::setRefTraj( ice_msgs::setCartPose::Request  & req ,
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
bool PR2adaptNeuroControllerClass::paramUpdate( ice_msgs::controllerParamUpdate::Request  & req ,
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

/// Service call to publish the saved data
bool PR2adaptNeuroControllerClass::publishExperimentData( std_srvs::Empty::Request & req,
                                                std_srvs::Empty::Response& resp )
{
	/* Then we can publish the buffer contents. */
	int  index;
	for (index = 0 ; index < storage_index_ ; index++)
	{
		//    pubFTData_         .publish(msgFTData         [index]);
		//    pubModelStates_    .publish(msgModelStates    [index]);
		//    pubRobotStates_    .publish(msgRobotStates    [index]);
		//    pubModelCartPos_   .publish(msgModelCartPos   [index]);
		//    pubRobotCartPos_   .publish(msgRobotCartPos   [index]);
		//    pubControllerParam_.publish(msgControllerParam[index]);
		//    pubControllerFullData_.publish(msgControllerFullData[index]);
		if(	experiment_ == PR2adaptNeuroControllerClass::A)
			pubExperimentDataA_.publish(experimentDataA_msg_[index]);
		if(	experiment_ == PR2adaptNeuroControllerClass::B)
			pubExperimentDataB_.publish(experimentDataB_msg_[index]);
		if(	experiment_ == PR2adaptNeuroControllerClass::C)
			pubExperimentDataC_.publish(experimentDataC_msg_[index]);
	}

	return true;
}

/// Service to check status
bool PR2adaptNeuroControllerClass::statusCB( ice_msgs::setBool::Request & req,
											 ice_msgs::setBool::Response& resp )
{
	resp.success = calibrateSensors;
	//resp.success = recordData;

	return true;
}

/// Service call to select sensor for Cailbration
bool PR2adaptNeuroControllerClass::tactileCalibrationCB(	ice_msgs::tactileCalibration::Request & req,
															ice_msgs::tactileCalibration::Response& resp )
{

	if(req.activeSensor < 0)	// Stop calibration
	{
		calibrateSensors = false;
		tactileSensorSelected_ = -1;

		for(int i=0; i<numTactileSensors_;i++)	// Fix weights
		{
			ARMAmodel_flexi_[i]->setUseFixedWeights(true);
		}

		resp.success = false;
		return true;
	}
	else	// Start calibration
	{
		calibrateSensors = true;
		tactileSensorSelected_ = req.activeSensor;

		maxCalibrationDistance_ = req.distance;
		calibrationDistance_ = 0.0;
		calibrationVelocity_ = req.distance / req.time;

		// Set start pose
		x0_cali_vec_ = affine2CartVec(x0_);	// Keep orientation
		x0_cali_vec_(0) = req.start.position.x;
		x0_cali_vec_(1) = req.start.position.y;
		x0_cali_vec_(2) = req.start.position.z;

		X_m = x0_cali_vec_;
		Xd_m.setZero();
		Xdd_m.setZero();

		x_r = x0_cali_vec_;
		xd_r.setZero();
		xdd_r.setZero();

		x0_cali_ = CartVec2Affine(x0_cali_vec_);
		refTrajSetForCalibration = false;

		// Select number of sensors to calibrate
		if(req.activeSensor < numTactileSensors_) // Single sensor
		{
			ARMAmodel_flexi_[tactileSensorSelected_]->setUseFixedWeights(false);
		}
		else	// All sensors
		{
			for(int i=0; i<numTactileSensors_;i++)
			{
				ARMAmodel_flexi_[i]->setUseFixedWeights(false);
			}
		}

		// Capture data
//		if(req.recordData)
//		{
//			experiment_ = PR2adaptNeuroControllerClass::B;
//			storage_index_ = 0;
//			recordData = true;
//		}
	}

	//resp.success = true;
	return true;
}

/// Service call set tactile filter Weights
bool PR2adaptNeuroControllerClass::tactileFilterWeightsCB(	ice_msgs::tactileFilterWeights::Request & req,
															ice_msgs::tactileFilterWeights::Response& resp )
{
	if(req.changeWeights)	// Modify weights
	{
		Eigen::MatrixXd tmp;
		tmp.resize(8,1);

		tmp <<	req.setWeights.f0,
				req.setWeights.f1,
				req.setWeights.f2,
				req.setWeights.f3,
				req.setWeights.f4,
				req.setWeights.f5,
				req.setWeights.f6,
				req.setWeights.f7;

		for(int i=0; i<numTactileSensors_;i++)
		{
			ARMAmodel_flexi_[i]->setWeights(tmp);
		}
	}
	else	// Return weights
	{
		if(0 <= req.sensor &&  req.sensor  < numTactileSensors_)
		{
			//Eigen::MatrixXd tmp;
			//ARMAmodel_flexi_[i]->getWeights(tmp);

			resp.getWeights.f0 = filterWeights_flexi_(0,req.sensor);
			resp.getWeights.f1 = filterWeights_flexi_(1,req.sensor);
			resp.getWeights.f2 = filterWeights_flexi_(2,req.sensor);
			resp.getWeights.f3 = filterWeights_flexi_(3,req.sensor);
			resp.getWeights.f4 = filterWeights_flexi_(4,req.sensor);
			resp.getWeights.f5 = filterWeights_flexi_(5,req.sensor);
			resp.getWeights.f6 = filterWeights_flexi_(6,req.sensor);
			resp.getWeights.f7 = filterWeights_flexi_(7,req.sensor);
		}
		else
		{
			resp.success = false;
		}
	}

	resp.success = true;
	return true;
}

/// Service call to run an experiment
bool PR2adaptNeuroControllerClass::runExperimentA(	ice_msgs::setValue::Request & req,
														ice_msgs::setValue::Response& resp )
{
	experiment_ = PR2adaptNeuroControllerClass::A;

	/* Record the starting time. */
	ros::Time started = ros::Time::now();

	// Re-set NN
	nnController.setUpdateWeights(false);
	Eigen::MatrixXd V_trans;
	Eigen::MatrixXd W_trans;
	//V_trans.setOnes( num_Hidden , num_Inputs + 1 ) ;
	V_trans.setRandom( num_Hidden , num_Inputs + 1 ) ;
	W_trans.setZero(   num_Outputs, num_Hidden     ) ;
	nnController.setInnerWeights(V_trans);
	nnController.setOuterWeights(W_trans);
	nnController.setUpdateWeights(true);

	// Set circle rate and decide if the inner weights will be updated
	if(req.value > 0)
	{
		nnController.setUpdateInnerWeights(true);
		circle_rate = req.value;
	}
	else
	{
		nnController.setUpdateInnerWeights(false);
		circle_rate = -req.value;
	}
	// Start circle traj
	executeCircleTraj = true;

	circle_velocity = circle_rate*circleAmpl;
	circle_phase = 0;
	loopsCircleTraj = 0;

	storage_index_ = 0;
	recordData = true;

	return true;
}

/// Service call to run an experiment
bool PR2adaptNeuroControllerClass::runExperimentB(	ice_msgs::setValue::Request & req,
													ice_msgs::setValue::Response& resp )
{
	experiment_ = PR2adaptNeuroControllerClass::B;

	/* Record the starting time. */
	ros::Time started = ros::Time::now();

	// Re-set NN
	nnController.setUpdateWeights(false);
	Eigen::MatrixXd V_trans;
	Eigen::MatrixXd W_trans;
	//V_trans.setOnes( num_Hidden , num_Inputs + 1 ) ;
	V_trans.setRandom( num_Hidden , num_Inputs + 1 ) ;
	W_trans.setZero(   num_Outputs, num_Hidden     ) ;
	nnController.setInnerWeights(V_trans);
	nnController.setOuterWeights(W_trans);
	nnController.setUpdateWeights(true);

	// Decide if the inner weights will be updated
	if(req.value > 0)
	{
		nnController.setUpdateInnerWeights(true);
	}
	else
	{
		nnController.setUpdateInnerWeights(false);
	}

	// Start with fixed ARMA weights
	for(int i=0; i<numTactileSensors_*tactile_dimensions_;i++)
	{
		ARMAmodel_flexi_[i]->setUseFixedWeights(true);
	}

	x_des_ = x0_;

	storage_index_ = 0;
	recordData = true;

	return true;
}

bool PR2adaptNeuroControllerClass::runExperimentC(	ice_msgs::setValue::Request & req,
													ice_msgs::setValue::Response& resp )
{
	experiment_ = PR2adaptNeuroControllerClass::C;

	/* Record the starting time. */
	ros::Time started = ros::Time::now();

	// Re-set NN
//	rbfnnController.
//	nnController.setUpdateWeights(false);
//	Eigen::MatrixXd V_trans;
//	Eigen::MatrixXd W_trans;
//	//V_trans.setOnes( num_Hidden , num_Inputs + 1 ) ;
//	V_trans.setRandom( num_Hidden , num_Inputs + 1 ) ;
//	W_trans.setZero(   num_Outputs, num_Hidden     ) ;
//	nnController.setInnerWeights(V_trans);
//	nnController.setOuterWeights(W_trans);
//	nnController.setUpdateWeights(true);
	// Decide if NN should be reset
	if(req.value < 0)
	{
		rbfnnController.resetWeights();
		circle_rate = -req.value;
	}
	else
	{
		circle_rate = req.value;
	}

	circle_phase = 0;
	storage_index_ = 0;
	recordData = true;

	return true;
}

/// Service call to capture and extract the data
bool PR2adaptNeuroControllerClass::capture(	std_srvs::Empty::Request & req,
                                                std_srvs::Empty::Response& resp )
{
//	/* Record the starting time. */
//	ros::Time started = ros::Time::now();
//
//	// Start circle traj
//	startCircleTraj = true;
//
//	/* Mark the buffer as clear (which will start storing). */
//	storage_index_ = 0;
//
//	/* Now wait until the buffer is full. */
//	while (storage_index_ < StoreLen)
//	{
//		/* Sleep for 1ms as not to hog the CPU. */
//		ros::Duration(0.001).sleep();
//
//		/* Make sure we don't hang here forever. */
//		if (ros::Time::now() - started > ros::Duration(20))
//		{
//			ROS_ERROR("Waiting for buffer to fill up took longer than 20 seconds!");
//			return false;
//		}
//	}
//
//	// Start circle traj
//	startCircleTraj = false;
//
//	/* Then we can publish the buffer contents. */
//	int  index;
//	for (index = 0 ; index < StoreLen ; index++)
//	{
//		//    pubFTData_         .publish(msgFTData         [index]);
//		//    pubModelStates_    .publish(msgModelStates    [index]);
//		//    pubRobotStates_    .publish(msgRobotStates    [index]);
//		//    pubModelCartPos_   .publish(msgModelCartPos   [index]);
//		//    pubRobotCartPos_   .publish(msgRobotCartPos   [index]);
//		//    pubControllerParam_.publish(msgControllerParam[index]);
//		//    pubControllerFullData_.publish(msgControllerFullData[index]);
//
//		pubExperimentDataA_.publish(experimentDataA_msg_[index]);
//	}

	// Capture data
	experiment_ = PR2adaptNeuroControllerClass::B;
	storage_index_ = 0;
	recordData = true;

	return true;
}

/// Service call to set reference trajectory
bool PR2adaptNeuroControllerClass::toggleFixedWeights( ice_msgs::fixedWeightToggle::Request & req,
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
void PR2adaptNeuroControllerClass::stopping()
{}


bool PR2adaptNeuroControllerClass::updateInnerNNweights( ice_msgs::setBool::Request& req,
		                                            ice_msgs::setBool::Response& resp )
{

	nnController.setUpdateInnerWeights(req.variable);
	resp.success = true;

	return true;
}
bool PR2adaptNeuroControllerClass::updateNNweights( ice_msgs::setBool::Request& req,
		                                            ice_msgs::setBool::Response& resp )
{

	nnController.setUpdateWeights(req.variable);
	resp.success = true;

	return true;
}
bool PR2adaptNeuroControllerClass::setNNweights( ice_msgs::setNNweights::Request& req,
		                                         ice_msgs::setNNweights::Response& resp )
{

    if( req.net.num_Inputs != num_Inputs || req.net.num_Hidden != num_Hidden || req.net.num_Outputs != num_Outputs )
    {
    	resp.success = false;
    }
    else
    {
    	Eigen::MatrixXd V_trans;
    	Eigen::MatrixXd W_trans;
		V_trans.resize( num_Hidden , num_Inputs + 1 ) ;
		W_trans.resize( num_Outputs, num_Hidden     ) ;

		int i = 0;
		for(int r=0;r<num_Hidden;r++)
		{
			for(int c=0;c<num_Inputs + 1;c++)
			{
				V_trans(r,c) = req.net.V.data[i];
				i++;
			}
		}

		i=0;
		for(int r=0;r<num_Outputs;r++)
		{
			for(int c=0;c<num_Hidden;c++)
			{
				W_trans(r,c) = req.net.W.data[i];
				i++;
			}
		}

		nnController.setInnerWeights(V_trans);
		nnController.setOuterWeights(W_trans);

		resp.success = true;
    }
	return true;
}
bool PR2adaptNeuroControllerClass::getNNweights( ice_msgs::getNNweights::Request& req,
		                                         ice_msgs::getNNweights::Response& resp )
{
    tf::matrixEigenToMsg(nnController.getInnerWeights(), resp.net.V);
    tf::matrixEigenToMsg(nnController.getOuterWeights(), resp.net.W);
    resp.net.num_Inputs = num_Inputs;
    resp.net.num_Hidden = num_Hidden;
    resp.net.num_Outputs = num_Outputs;

	return true;
}

void
PR2adaptNeuroControllerClass::calcHumanIntentPos( Eigen::Vector3d & force,
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

void PR2adaptNeuroControllerClass::readForceValuesCB(const geometry_msgs::WrenchStampedConstPtr& wrench_msg)
{
	// Convert to CartVec
	tactile_wrench_(0) = wrench_msg->wrench.force.x;
	tactile_wrench_(1) = wrench_msg->wrench.force.y;
	tactile_wrench_(2) = wrench_msg->wrench.force.z;
	tactile_wrench_(3) = wrench_msg->wrench.torque.x;
	tactile_wrench_(4) = wrench_msg->wrench.torque.y;
	tactile_wrench_(5) = wrench_msg->wrench.torque.z;
}

void PR2adaptNeuroControllerClass::readTactileDataCB(const ice_msgs::tactileArrayDataConstPtr& msg)
{
	// Convert to CartVec
	for( int i=0; i<msg->data.size(); i++)
	{
		tactile_data_[i] = msg->data[i];
	}
}

bool PR2adaptNeuroControllerClass::initParam()
{
	nh_.param("/loopRateFactor",    loopRateFactor,    3);

	nh_.param("/calibrateSensors",  calibrateSensors,  false);
	nh_.param("/max_calibration_distance", maxCalibrationDistance_,     1.0);

	nh_.param("/forceTorqueOn",     forceTorqueOn,     false);
	nh_.param("/accelerometerOn",   accelerometerOn,   false);
	nh_.param("/useFlexiForce",     useFlexiForce,     false);
	nh_.param("/executeCircleTraj", executeCircleTraj, false);
	nh_.param("/numCircleTraj",     numCircleTraj,     2);
	nh_.param("/publishRTtopics",   publishRTtopics,   false);
	nh_.param("/useDigitalFilter",  useDigitalFilter,  false);

	nh_.param("/useOuterloop",      useOuterloop,      false);

	nh_.param("/mannequinThresPos", mannequinThresPos, 0.05);
	nh_.param("/mannequinThresPos", mannequinThresRot, 0.05);
	nh_.param("/mannequinMode",     mannequinMode,     false);
	nh_.param("/useHumanIntent",    useHumanIntent,   false);

	int tmp;
	nh_.param("/experiment", tmp, 1);		// TODO check user input
	experiment_ = (PR2adaptNeuroControllerClass::Experiment)tmp;
	ROS_INFO("Experiment selected: %i",experiment_);

	return true;
}

bool PR2adaptNeuroControllerClass::initRobot()
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

	// Determine arm from tip_name
	arm_prefix = tip_name[0];
	if( arm_prefix == "r" || arm_prefix == "l")
		ROS_INFO("Arm prefix: %s",arm_prefix.c_str());
	else
		ROS_ERROR("Unknown arm (l or r) from tip name: %s",arm_prefix.c_str());


	// Construct a chain from the root to the tip and prepare the kinematics.
	if (!chain_.init(robot_state_, root_name, tip_name))
	{
		ROS_ERROR("The controller could not use the chain from '%s' to '%s'", root_name.c_str(), tip_name.c_str());
		return false;
	}

	// Check that joints are calibrated
	if (!chain_.allCalibrated())
	{
		ROS_ERROR("Joints are not calibrated in given namespace: %s)",nh_.getNamespace().c_str());
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
	num_Joints = kdl_chain_.getNrOfJoints();				// TODO replace kdl_chain_.getNrOfJoints() with Joints
	tau_.resize(num_Joints);
	tau_c_.resize(num_Joints);
	tau_c_latest_.resize(num_Joints);
	tau_measured_.resize(num_Joints);

	for(int i=0;i<num_Joints;i++)
	{
		tau_c_(i) = 0.0;
		tau_c_latest_(i) = 0.0;
	}

	qnom.resize(num_Joints);
	q_lower.resize(num_Joints);
	q_upper.resize(num_Joints);
	qd_limit.resize(num_Joints);

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

	/* Sensors */

	// Get gripper accelerometer tip name and construct a chain
	std::string gripper_acc_tip;
	if (!nh_.getParam("/gripper_acc_tip", gripper_acc_tip))
	{
		ROS_ERROR("No accelerometer tip name given in namespace: %s)",nh_.getNamespace().c_str());
		return false;
	}
	if (!chain_acc_link.init(robot_state_, root_name, gripper_acc_tip))
	{
		ROS_ERROR("The controller could not use the chain from '%s' to '%s'", root_name.c_str(), gripper_acc_tip.c_str());
		return false;
	}

	// Get FT frame ID
	if (!nh_.getParam("/ft_frame_id", ft_frame_id))
	{
		ROS_ERROR("No ft_frame_id name given in namespace: %s)",nh_.getNamespace().c_str());
		ft_frame_id = root_name;
		return false;
	}

	// Construct kdl solvers for F/T frames
	if(forceTorqueOn)
	{
		// Torso to FT frame
		if (!chain_ft_link.init(robot_state_, root_name, ft_frame_id))
		{
			ROS_ERROR("The controller could not use the chain from '%s' to '%s'", root_name.c_str(), ft_frame_id.c_str());
			return false;
		}
		chain_ft_link.toKDL(kdl_chain_ft_link);
		kin_ft_.reset(new Kin<Joints>(kdl_chain_ft_link));

		// Accel to FT frame
		if (!chain_acc_to_ft_link.init(robot_state_, gripper_acc_tip, ft_frame_id))
		{
			ROS_ERROR("The controller could not use the chain from '%s' to '%s'", gripper_acc_tip.c_str(), ft_frame_id.c_str());
			return false;
		}
		chain_acc_to_ft_link.toKDL(kdl_chain_acc_to_ft_link);
		kin_acc_to_ft_.reset(new Kin<Joints>(kdl_chain_acc_to_ft_link));
	}

	// Torque Saturation
	sat_scaling = 1.0;
	tau_sat.resize( 7 );		// num_Outputs

	// Joint velocity filter
	nh_.param("/joint_vel_filter", joint_vel_filter_,     1.0);


	return true;
}

bool PR2adaptNeuroControllerClass::initTrajectories()
{

	bool success = true;

	// Set desired trajectories to zero
	CartVec tmp;
	tmp.setZero();
	x_des_   = CartVec2Affine(tmp);	// Position
	xd_des_  = CartVec2Affine(tmp); // Velocity
	xdd_des_ = CartVec2Affine(tmp); // Acceleration

	// Rate for circle trajectory
	circle_rate = 3  ;
	circleLlim  = 0  ;
	circleUlim  = 1.5;

	std::string para_circleRate  = "/circleRate" ;
	std::string para_circleLlim  = "/circleLlim" ;
	std::string para_circleUlim  = "/circleUlim" ;

	if (!nh_.getParam( para_circleRate , circle_rate )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_circleRate .c_str()) ; success = false; }
	if (!nh_.getParam( para_circleLlim , circleLlim  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_circleLlim .c_str()) ; success = false; }
	if (!nh_.getParam( para_circleUlim , circleUlim  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_circleUlim .c_str()) ; success = false; }

	loopsCircleTraj = 0;
	circleAmpl = (circleUlim - circleLlim)/2 ;
	circle_velocity = circle_rate*circleAmpl;

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

	if (!nh_.getParam( para_cartDesX     , cartDesX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesX    .c_str()) ; success = false; }
	if (!nh_.getParam( para_cartDesY     , cartDesY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesY    .c_str()) ; success = false; }
	if (!nh_.getParam( para_cartDesZ     , cartDesZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesZ    .c_str()) ; success = false; }
	if (!nh_.getParam( para_cartDesRoll  , cartDesRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesRoll .c_str()) ; success = false; }
	if (!nh_.getParam( para_cartDesPitch , cartDesPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesPitch.c_str()) ; success = false; }
	if (!nh_.getParam( para_cartDesYaw   , cartDesYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesYaw  .c_str()) ; success = false; }


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

	if (!nh_.getParam( para_cartIniX     , cartIniX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniX.c_str()) ; success = false; }
	if (!nh_.getParam( para_cartIniY     , cartIniY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniY.c_str()) ; success = false; }
	if (!nh_.getParam( para_cartIniZ     , cartIniZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniZ.c_str()) ; success = false; }
	if (!nh_.getParam( para_cartIniRoll  , cartIniRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniRoll .c_str()) ; success = false; }
	if (!nh_.getParam( para_cartIniPitch , cartIniPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniPitch.c_str()) ; success = false; }
	if (!nh_.getParam( para_cartIniYaw   , cartIniYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniYaw  .c_str()) ; success = false; }


	return success;
}

bool PR2adaptNeuroControllerClass::initSensors()
{
	bool result=true;

	// Get a handle to the hardware interface
	pr2_hardware_interface::HardwareInterface* hardwareInterface = robot_state_->model_->hw_;
	if(!hardwareInterface)
	{
		ROS_ERROR("Something wrong with the hardware interface pointer!");
		result=false;
	}


	if( forceTorqueOn )
	{
		// Get FT handles
		ft_handle_ = hardwareInterface->getForceTorque(arm_prefix+"_gripper_motor");

		if(!ft_handle_)
		{
			ROS_ERROR("Something wrong with getting ft handle!");
			result=false;
		}

		// Load FT bias parameter
		ft_bias.setZero();
		std::vector<double> bias_list;
		if(!nh_.getParam("/bias", bias_list))
		{
			ROS_ERROR("Value not loaded from parameter: /bias !)");
			result=false;
		}
		else
		{
			if( bias_list.size() != 6)
			{
				ROS_ERROR("Bias vector has wrong size!");
				result=false;
			}
			else
			{
				for(unsigned i=0; i < bias_list.size(); i++)
				{
					ft_bias(i) = bias_list[i];
				}
			}
		}
		// Load gripper COM pose parameter
		r_gripper_com.setZero();
		std::vector<double> gripper_com_list;
		if(!nh_.getParam("/gripper_com_pose", gripper_com_list))
		{
			ROS_ERROR("Value not loaded from parameter: /gripper_com_pose !)");
			result=false;
		}
		else
		{
			if( gripper_com_list.size() != 6)
			{
				ROS_ERROR("Gripper vector has wrong size");
				result=false;
			}
			else
			{
				for(unsigned i=0; i < 3; i++)
				{
					r_gripper_com(i) = gripper_com_list[i];
				}
			}
		}
		// Load gripper mass parameter
		if (!nh_.getParam( "/gripper_mass", gripper_mass))
		{
			ROS_ERROR("Value not loaded from parameter: /gripper_mass !)") ;
			gripper_mass = 0.0;
			result=false;
		}

	}

	if( useDigitalFilter )
	{
		int num_coeff;
		// Load filter coefficients	(denominator)
		a_filt.setZero();
		std::vector<double> a_filt_list;
		if(!nh_.getParam("/a_filt", a_filt_list))
		{
			ROS_ERROR("Value not loaded from parameter: /a_filt !)");
			result=false;
		}
		else
		{
			num_coeff = a_filt_list.size();
			a_filt.resize(num_coeff);
			for(unsigned i=0; i < num_coeff; i++)
			{
				a_filt(i) = a_filt_list[i];
			}
		}
        std::cout<<"Loaded /a_filt: \n"<<a_filt<<"\n---\n";
		// Load filter coefficients	(numerator)
		b_filt.setZero();
		std::vector<double> b_filt_list;
		if(!nh_.getParam("/b_filt", b_filt_list))
		{
			ROS_ERROR("Value not loaded from parameter: /b_filt !)");
			result=false;
		}
		else
		{
			num_coeff = b_filt_list.size();		// should be the same
			b_filt.resize(num_coeff);
			for(unsigned i=0; i < num_coeff; i++)
			{
				b_filt(i) = b_filt_list[i];
			}
		}
        std::cout<<"Loaded /b_filt: \n"<<b_filt<<"\n---\n";
        
        int order = num_coeff-1;
        std::cout<<"Filter order: "<<order<<"\n";
        for(int i=0;i<6;i++)
        {
        	digitalFilter tmp;
        	if(!tmp.init(order, true, b_filt, a_filt))
    		{
    			ROS_ERROR("Failed to init digital filter!");
    			result=false;
    		}
        	digitalFilters.push_back(tmp);
        }

	}

	if( accelerometerOn )//|| forceTorqueOn )
	{
		// Get accelerometer handles
		accelerometer_handle_ = hardwareInterface->getAccelerometer(arm_prefix+"_gripper_motor");
		if(!accelerometer_handle_)
		{
			ROS_ERROR("Something wrong with getting accelerometer handle!");
			result=false;
		}

		// set to 1.5 kHz bandwidth (should be the default)
//		accelerometer_handle_->command_.bandwidth_ = pr2_hardware_interface::AccelerometerCommand::BANDWIDTH_1500HZ;	// TODO check if needed

		// set to +/- 8g range (0=2g,1=4g)
//		accelerometer_handle_->command_.range_ = pr2_hardware_interface::AccelerometerCommand::RANGE_8G;	// TODO check if needed

		//accObserver = new accelerationObserver(accelerometer_handle_);	// FIXME This seems to crash the RT loop
	}

	// Tactile sensors
	nh_.param("/num_tactile_sensors",     numTactileSensors_,     4);
	tactileSensorSelected_ = 0;
	calibrateSingelSensors = true;

	if(useFlexiForce)
	{
		sensorDirections.resize(3,numTactileSensors_);
		sensorDirections.setZero();

		for(int i=0;i<numTactileSensors_;i++)
		{
			std::string name = "/sensor_"+boost::lexical_cast<std::string>(i);

			std::vector<double> tmp;
			if(!nh_.getParam(name, tmp))
			{
				ROS_ERROR("Value not loaded from parameter: %s !)", name.c_str()) ;
				result=false;
			}
			else
			{
				if(tmp.size() != 3)
				{
					ROS_ERROR("Unexpected length from parameter: %s !)", name.c_str()) ;
					result=false;
				}
				else
				{
					for(int d=0; d < 3; d++)
					{
						sensorDirections(d,i) = tmp[d];
					}
				}

			}
		}
		std::cout<<"Loaded sensor directions: "<<sensorDirections<<"\n---\n";
	}

	Vec3d_ones.setOnes();
	calibrationCounter = -1;

	return result;
}

bool PR2adaptNeuroControllerClass::initNN()
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


bool PR2adaptNeuroControllerClass::initOuterLoop()
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


	// TODO make filter weights vectors
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
	outerLoopWk_flexi_0.resize(8,1);
	outerLoopWk_flexi_1.resize(8,1);
	outerLoopWk_flexi_2.resize(8,1);
	outerLoopWk_flexi_3.resize(8,1);


	if( useFIRmodel || useARMAmodel || useCTARMAmodel)
	{
		outerLoopWk(0,0) = filtW0 ; outerLoopWk_flexi_0(0,0) = flex_1_filtW0 ; outerLoopWk_flexi_1(0,0) = flex_2_filtW0 ; outerLoopWk_flexi_2(0,0) = flex_3_filtW0 ; outerLoopWk_flexi_3(0,0) = flex_4_filtW0 ;
		outerLoopWk(1,0) = filtW1 ; outerLoopWk_flexi_0(1,0) = flex_1_filtW1 ; outerLoopWk_flexi_1(1,0) = flex_2_filtW1 ; outerLoopWk_flexi_2(1,0) = flex_3_filtW1 ; outerLoopWk_flexi_3(1,0) = flex_4_filtW1 ;
		outerLoopWk(2,0) = filtW2 ; outerLoopWk_flexi_0(2,0) = flex_1_filtW2 ; outerLoopWk_flexi_1(2,0) = flex_2_filtW2 ; outerLoopWk_flexi_2(2,0) = flex_3_filtW2 ; outerLoopWk_flexi_3(2,0) = flex_4_filtW2 ;
		outerLoopWk(3,0) = filtW3 ; outerLoopWk_flexi_0(3,0) = flex_1_filtW3 ; outerLoopWk_flexi_1(3,0) = flex_2_filtW3 ; outerLoopWk_flexi_2(3,0) = flex_3_filtW3 ; outerLoopWk_flexi_3(3,0) = flex_4_filtW3 ;
		outerLoopWk(4,0) = filtW4 ; outerLoopWk_flexi_0(4,0) = flex_1_filtW4 ; outerLoopWk_flexi_1(4,0) = flex_2_filtW4 ; outerLoopWk_flexi_2(4,0) = flex_3_filtW4 ; outerLoopWk_flexi_3(4,0) = flex_4_filtW4 ;
		outerLoopWk(5,0) = filtW5 ; outerLoopWk_flexi_0(5,0) = flex_1_filtW5 ; outerLoopWk_flexi_1(5,0) = flex_2_filtW5 ; outerLoopWk_flexi_2(5,0) = flex_3_filtW5 ; outerLoopWk_flexi_3(5,0) = flex_4_filtW5 ;
		outerLoopWk(6,0) = filtW6 ; outerLoopWk_flexi_0(6,0) = flex_1_filtW6 ; outerLoopWk_flexi_1(6,0) = flex_2_filtW6 ; outerLoopWk_flexi_2(6,0) = flex_3_filtW6 ; outerLoopWk_flexi_3(6,0) = flex_4_filtW6 ;
		outerLoopWk(7,0) = filtW7 ; outerLoopWk_flexi_0(7,0) = flex_1_filtW7 ; outerLoopWk_flexi_1(7,0) = flex_2_filtW7 ; outerLoopWk_flexi_2(7,0) = flex_3_filtW7 ; outerLoopWk_flexi_3(7,0) = flex_4_filtW7 ;
	}else
	{
		outerLoopWk.setZero();
		outerLoopWk_flexi_0.setZero();
		outerLoopWk_flexi_1.setZero();
		outerLoopWk_flexi_2.setZero();
		outerLoopWk_flexi_3.setZero();
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

	q       .resize( num_Joints ) ;
	qd      .resize( num_Joints ) ;
	qdd     .resize( num_Joints ) ;

	q_m     .resize( num_Joints ) ;
	qd_m    .resize( num_Joints ) ;
	qdd_m   .resize( num_Joints ) ;

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
	tau     .resize( num_Joints ) ;

	q        .setZero() ;
	qd       .setZero() ;
	qdd      .setZero() ;
	q_m      .setZero() ;
	qd_m     .setZero() ;
	qdd_m    .setZero() ;

	X_m      .setZero() ;
	Xd_m     .setZero() ;
	Xdd_m    .setZero() ;
	X        .setZero() ;
	Xd       .setZero() ;

	p_X_m    .setZero() ;
	p_Xd_m   .setZero() ;
	p_Xdd_m  .setZero() ;

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
	accData           = Eigen::Vector3d::Zero();

	x_d                  = Eigen::VectorXd::Zero( num_Outputs ) ;
	prev_x_d             = Eigen::VectorXd::Zero( num_Outputs ) ;

	t_r                  = Eigen::VectorXd::Zero( num_Outputs ) ;
	task_ref             = Eigen::VectorXd::Zero( num_Outputs ) ;
	x_r                  = Eigen::VectorXd::Zero( num_Outputs ) ;
	xd_r                 = Eigen::VectorXd::Zero( num_Outputs ) ;
	xdd_r                = Eigen::VectorXd::Zero( num_Outputs ) ;
	prev_x_r             = Eigen::VectorXd::Zero( num_Outputs ) ;
	delta_x				 = 0;
	task_refModel_output = Eigen::VectorXd::Zero( num_Outputs ) ;
	tau                  = Eigen::VectorXd::Zero( num_Outputs ) ;
	force_c              = Eigen::VectorXd::Zero( num_Outputs ) ;

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

	outerLoopRLSmodelX.updateDelT( outerLoopTime );		// FIXME assumens 1D, 4+4*4 filter parameters
	outerLoopRLSmodelX.updateAB( task_mA, task_mB );
	outerLoopRLSmodelX.initRls( rls_lambda, rls_sigma );
	//  outerLoopRLSmodelX.initPos( cartIniX );


	outerLoopRLSmodelY.updateDelT( outerLoopTime );
	outerLoopRLSmodelY.updateAB( task_mA, task_mB );
	outerLoopRLSmodelY.initRls( rls_lambda, rls_sigma );
	//  outerLoopRLSmodelY.initPos( cartIniY );

	weightsRLSmodelX.resize(4+1*4,1);	// FIXME use parameters
	weightsRLSmodelY.resize(4+1*4,1);
	weightsRLSmodelX.setZero();
	weightsRLSmodelY.setZero();

	// ARMA models for each tactile sensor
	tactile_data_.resize(numTactileSensors_);
	tactile_data_.setZero();
	filterWeights_flexi_.resize(8,numTactileSensors_);		// TODO use number of filter weights
	filterWeights_flexi_.setZero();
	for(int i=0;i<numTactileSensors_;i++)
	{
		csl::outer_loop::RlsModel* tmpPtr = new csl::outer_loop::RlsModel(1, 4, 4, 1);
		tmpPtr->updateDelT( outerLoopTime );
		tmpPtr->updateAB( task_mA, task_mB );
		tmpPtr->initRls( rls_lambda, rls_sigma );
		tmpPtr->setUseFixedWeights(true);

		ARMAmodel_flexi_.push_back(tmpPtr);
	}

	// One ARMA model for all the sensors
	ARMAmodel_flexi_combined_ = new csl::outer_loop::RlsModel(1, 4, 4, 4);
	ARMAmodel_flexi_combined_->updateDelT( outerLoopTime );
	ARMAmodel_flexi_combined_->updateAB( task_mA, task_mB );
	ARMAmodel_flexi_combined_->initRls( rls_lambda, rls_sigma );
	ARMAmodel_flexi_combined_->setUseFixedWeights(true);

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

bool PR2adaptNeuroControllerClass::initInnerLoop()
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

	// Default values
	num_Inputs  = 44 ;
	num_Outputs = 6  ; // Cartesian
	num_Hidden  = 5  ;
	num_Error   = 6  ;
	num_Joints  = 7  ; // 7 DOF

	if (!nh_.getParam( para_nnNum_Inputs , num_Inputs  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Inputs .c_str()) ; return false; }
	if (!nh_.getParam( para_nnNum_Outputs, num_Outputs )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Outputs.c_str()) ; return false; }
	if (!nh_.getParam( para_nnNum_Hidden , num_Hidden  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Hidden .c_str()) ; return false; }
	if (!nh_.getParam( para_nnNum_Error  , num_Error   )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Error  .c_str()) ; return false; }
	if (!nh_.getParam( para_nnNum_Joints , num_Joints  )) { ROS_ERROR("Value not loaded from parameter: %s !)", para_nnNum_Joints .c_str()) ; return false; }

	return true;
}

bool PR2adaptNeuroControllerClass::initNullspace()
{
	IdentityCart.setIdentity();
	IdentityJoints.setIdentity();

	return true;
}
