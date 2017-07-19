#include <ice_robot_controllers/adaptNeuroController.h>
#include <pluginlib/class_list_macros.h>

using namespace pr2_controller_ns;

template<typename Derived>
inline bool is_finite(const Eigen::MatrixBase<Derived>& x)
{
	return ( (x - x).array() == (x - x).array()).all();
}

template<typename Derived>
inline bool is_nan(const Eigen::MatrixBase<Derived>& x)
{
	return ((x.array() == x.array())).all();
}

// Register controller to pluginlib
PLUGINLIB_EXPORT_CLASS( pr2_controller_ns::PR2adaptNeuroControllerClass, pr2_controller_interface::Controller)

PR2adaptNeuroControllerClass::PR2adaptNeuroControllerClass()
: robot_state_(NULL)			// Initialize variables
{
}

PR2adaptNeuroControllerClass::~PR2adaptNeuroControllerClass()
{
	m_Thread.interrupt();		// Kill thread at one of the interruption points

	for(int i=0; i<ARMAmodel_flexi_.size();i++)
	{
		if(ARMAmodel_flexi_[i])
			delete ARMAmodel_flexi_[i];
	}
	for(int i=0; i<ARMAmodel_FT_.size();i++)
	{
		if(ARMAmodel_FT_[i])
			delete ARMAmodel_FT_[i];
	}
	if(ptrNNController)
		delete ptrNNController;
	if(ptrNNController)
		delete ptrJTController;
	if(ptrNNEstimator)
		delete ptrNNEstimator;
}

/// Controller initialization in non-realtime
bool PR2adaptNeuroControllerClass::init(pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n)
{
	ROS_INFO("Initializing Neuroadaptive Controller...");

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

	// JT Controller
	ptrJTController = new JTCartesianController(n);

	// Subscribe to Flexiforce wrench commands
	if(useFlexiForce)
	{
		sub_tactileWrench_ = nh_.subscribe<geometry_msgs::WrenchStamped>("/tactile/wrench", 1, &PR2adaptNeuroControllerClass::readForceValuesCB, this);
		sub_tactileData_   = nh_.subscribe<ice_msgs::tactileArrayData>("/tactile/data", 1, &PR2adaptNeuroControllerClass::readTactileDataCB, this);
	}
	tactileCalibration_srv_ = nh_.advertiseService("/tactile/calibration" , &PR2adaptNeuroControllerClass::tactileCalibrationCB   , this);
	status_srv_ = nh_.advertiseService("/tactile/status" , &PR2adaptNeuroControllerClass::statusCB   , this);
	tactileFilterWeights_srv_ = nh_.advertiseService("/tactile/filterWeights" , &PR2adaptNeuroControllerClass::tactileFilterWeightsCB       , this);

	if(externalRefTraj)
	{
		sub_commandPose_ = nh_.subscribe("command_pose", 1, &PR2adaptNeuroControllerClass::commandPoseCB, this);
	}

	runExperimentA_srv_ = nh_.advertiseService("runExperimentA" , &PR2adaptNeuroControllerClass::runExperimentA   , this);
	runExperimentB_srv_ = nh_.advertiseService("runExperimentB" , &PR2adaptNeuroControllerClass::runExperimentB   , this);
	runExperimentC_srv_ = nh_.advertiseService("runExperimentC" , &PR2adaptNeuroControllerClass::runExperimentC   , this);
	runExperimentD_srv_ = nh_.advertiseService("runExperimentD" , &PR2adaptNeuroControllerClass::runExperimentD   , this);

	setNNparam_srv_  = nh_.advertiseService("setNNparam" , &PR2adaptNeuroControllerClass::setNNparamCB, this);

	// NN weights
	updateInnerNNweights_srv_  = nh_.advertiseService("updateInnerNNweights" , &PR2adaptNeuroControllerClass::updateInnerNNweights   , this);
	updateNNweights_srv_  = nh_.advertiseService("updateNNweights" , &PR2adaptNeuroControllerClass::updateNNweights   , this);
	setNNweights_srv_  = nh_.advertiseService("setNNweights" , &PR2adaptNeuroControllerClass::setNNweights   , this);
	getNNweights_srv_  = nh_.advertiseService("getNNweights" , &PR2adaptNeuroControllerClass::getNNweights   , this);

	/////////////////////////
	// DATA COLLECTION
	publish_srv_             = nh_.advertiseService("publishExpData"     , &PR2adaptNeuroControllerClass::publishExperimentData, this);
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
	pubExperimentDataA_	     = nh_.advertise< ice_msgs::experimentDataA    >( "experimentDataA"      , StoreLen);
	pubExperimentDataB_	     = nh_.advertise< ice_msgs::experimentDataB    >( "experimentDataB"      , StoreLen);
	pubExperimentDataC_	     = nh_.advertise< ice_msgs::experimentDataC    >( "experimentDataC"      , StoreLen);
	pubExperimentDataState_  = nh_.advertise< StateMsg                     >( "experimentDataState"  , StoreLen);

	storage_index_ = StoreLen;
	// DATA COLLECTION END
	/////////////////////////

	StateMsg state_template;
	state_template.header.frame_id = root_name;
	state_template.missed_updates_counter = 0;
	state_template.x.header.frame_id = root_name;
	state_template.x_desi.header.frame_id = root_name;
	state_template.x_desi_filtered.header.frame_id = root_name;
	state_template.q.resize(Joints);
	state_template.tau_c.resize(Joints);
	state_template.tau_posture.resize(Joints);
//	state_template.J.layout.dim.resize(2);
//	state_template.J.data.resize(6*Joints);
//	state_template.N.layout.dim.resize(2);
//	state_template.N.data.resize(Joints*Joints);
	// NN weights
//	state_template.W.layout.dim.resize(2);
//	state_template.W.data.resize(num_Outputs*num_Hidden);
//	state_template.V.layout.dim.resize(2);
//	state_template.V.data.resize(num_Hidden*(num_Inputs+1));
	pub_state_.init(nh_, "state", 1);
	pub_state_.lock();
	pub_state_.msg_ = state_template;
	pub_state_.unlock();

	pub_x_desi_.init(nh_, "tmp_data", 1);
	pub_x_desi_.lock();
	pub_x_desi_.msg_.header.frame_id = root_name;
	pub_x_desi_.unlock();

	pub_ft_.init(nh_, "ft/l_gripper",1);
	pub_ft_.lock();
	pub_ft_.msg_.header.frame_id = ft_frame_id;
	pub_ft_.unlock();

	pub_ft_transformed_.init(nh_, "ft/l_gripper_transformed",1);
	pub_ft_transformed_.lock();
	pub_ft_transformed_.msg_.header.frame_id = root_name;
	pub_ft_transformed_.unlock();

	ROS_INFO("Neuroadpative Controller is initialized!");
	return true;
}


/// Controller startup in realtime
void PR2adaptNeuroControllerClass::starting()
{
	ROS_INFO("Starting Neuroadaptive Controller...");

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
	x0_vec_ = affine2CartVec(x0_);
	x_des_ = x0_;
	CartVec cv = affine2CartVec(x_des_);
	PoseVec pv = affine2PoseVec(x_des_);
	ROS_INFO("Starting pose: pos=[%f,%f,%f], rot=[%f,%f,%f]",cv(0),cv(1),cv(2),cv(3),cv(4),cv(5));
	ROS_INFO("Starting pose: pos=[%f,%f,%f], rot=[%f,%f,%f,%f]",pv(0),pv(1),pv(2),pv(3),pv(4),pv(5),pv(6));

	ptrJTController->starting(x0_);

	// Reference trajectory
	convert2NNinput(x_des_, X_m);	// Assume xd, xdd are zero
	p_X_m = X_m;
	ROS_INFO_STREAM("Desired position: "<< X_m.transpose());
	commandPose = x_des_;

	// Set transform from accelerometer to FT sensor
	CartVec tmp;
	tmp << 0, 0, 0, 3.142, 1.571, 1.919;	// rosrun tf tf_echo l_force_torque_link l_gripper_motor_accelerometer_link
	x_acc_to_ft_ = CartVec2Affine(tmp);		// TODO: get from robot or load from config file

	// Set filter values
	qdot_filtered_.setZero();

	loop_count_ = 0;
	missed_updates_count_= 0;
	invalid_value_count_ = 0;
	recordData = false;

	nne_scaling = 0.0;

	// Start threads
	runComputations = false;
	m_Thread = boost::thread(&PR2adaptNeuroControllerClass::updateNonRealtime, this);

}


/// Controller stopping in realtime
void PR2adaptNeuroControllerClass::stopping()
{
	runComputations = false;
	m_Thread.interrupt();		// Kill thread at one of the interruption points
}

/// Parallel thread for updates that are computationally expensive
void PR2adaptNeuroControllerClass::updateNonRealtime()
{
	while(true)
	{
		// 1) Wait
		boost::mutex::scoped_lock lock(m_Mutex);
		while(!runComputations)
		{
			m_Condition.wait(lock);		// Note: this is also an interrupt point
			//boost::this_thread::sleep(boost::posix_time::microseconds(1));
		}

		// 2) Do Computations
		//invalid_value_count_ = 0;

		/***************** UPDATE LOOP VARIABLES *****************/

		JacobianTrans = J_.transpose();			// [6x7]^T->[7x6]

		//force_c.setZero();		// [6x1] (num_Outputs)
		tau_   .setZero();		// [7x1] (num_Joints)

		// Current joint positions and velocities
		q = q_;
		qd = qdot_;

		//X = affine2CartVec(x_);
		convert2NNinput(x_, X);
		convert2NNinput(xdot_, Xd);

		Xd_m.setZero();
		Xdd_m.setZero();

		/***************** SENSOR DATA PROCESSING *****************/

		if( forceTorqueOn )		// TODO check if accData has been updated
		{
			wrench_transformed_prev_ = wrench_transformed_;

			wrench_gripper_.topRows(3) = gripper_mass * (x_acc_to_ft_.linear()*accData);	// Gripper force due to gravity

			forceFT =  wrench_gripper_.topRows(3);	// Temporary store values due to Eigen limitations

			wrench_gripper_.bottomRows(3) = r_gripper_com.cross(forceFT); // Torque vector

			wrench_compensated_ = wrenchData - ft_bias - wrench_gripper_;

			forceFT = wrench_compensated_.topRows(3);
			tauFT	= wrench_compensated_.bottomRows(3);

			// Force transformation

			//W_mat_(0,0) = 0;
			W_mat_(0,1) = -x_ft_.translation().z();
			W_mat_(0,2) = x_ft_.translation().y();

			W_mat_(1,0) = x_ft_.translation().z();
			//W_mat_(1,1) = 0;
			W_mat_(1,2) = -x_ft_.translation().x();

			W_mat_(2,0) = -x_ft_.translation().y();
			W_mat_(2,1) = x_ft_.translation().x();
			//W_mat_(2,2) = 0;

			wrench_transformed_.bottomRows(3) = W_mat_*x_ft_.linear()*forceFT + x_ft_.linear()*tauFT;
			wrench_transformed_.topRows(3)    = x_ft_.linear()*forceFT;

			// Check for valid values
			for(int i=0;i<wrench_transformed_.size();i++)
			{
				if( isnan(wrench_transformed_(i)) )
				{
					wrench_transformed_(i) = wrench_transformed_prev_(i);
					invalid_value_count_ += 1;
				}
				if( isinf(wrench_transformed_(i)) )
				{
					wrench_transformed_(i) = wrench_transformed_prev_(i);
					invalid_value_count_ += 10;
				}
			}

			// Apply low-pass filter
			if(useDigitalFilter)		// FIXME wrench_filtered_ not updated if false
			{
		        for(int i=0;i<6;i++)
		        {
		        	//wrench_filtered_(i) = digitalFilters[i].getNextFilteredValue(wrench_transformed_(i));
		        	wrench_filtered_(i) += ft_filter_ * ( wrench_transformed_(i) - wrench_filtered_(i));
		        }
			}
			transformed_force = wrench_filtered_.topRows(3);
		}
		else
		{
			wrench_transformed_.setZero();
		}

		if(useFlexiForce)
		{
			//tactile_wrench_ = -tactile_wrench_;
			// TODO: transform into torso frame
			transformed_force = tactile_wrench_.topRows(3);		// this variable is being updated by the readForceValuesCB
		}

		// Feedforward force, human force [6x1]
		if(useDigitalFilter)
			convert2NNinput(wrench_filtered_,force_h);
		else
			convert2NNinput(wrench_transformed_,force_h);

		// Force scaling factor
		if(useForceScaling)
		{
			force_h = force_h.cwiseProduct(forceScaling);
		}

		// Force threshold (makes force zero bellow threshold)
		if(useForceCutOff)
		{
			force_h = (force_h.array().abs() < forceCutOff.array()).select(Eigen::VectorXd::Zero(6), force_h);
		}

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
				CartVec p = affine2CartVec(x_des_);
				ROS_INFO("End pose: pos=[%f,%f,%f], rot=[%f,%f,%f]",p(0),p(1),p(2),p(3),p(4),p(5));
			}
		}
		if(mannequinMode && loop_count_ > 10) // Check if initialized
		{

			// Compute error
//			computePoseError(x_, x_des_, xerr_);
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

//			if(xerr_.norm() > mannequinThresPos)	// TODO: implement two threshold
//			{
//				x_des_ = x_;
//			}
/*
			if( fabs( x_.translation().z() -  x_des_.translation().z() ) > mannequinThresPos)	// Only change z //
			{
				CartVec tmp;
				tmp << 0.65, 0.35, x_.translation().z(), 1.57, 0.0, 0.0;
				x_des_ = CartVec2Affine(tmp);

				// x_des_.translation().z() = x_.translation().z();  <- oscillates?
			}
*/
			X_m.setZero();
			X_m.head(2) = X.head(2);
			X_m(2) = -0.05;
			x_des_  = CartVec2Affine(X_m);
		}
		if(useHumanIntent && loop_count_ > 100) // TODO make sure this is executed before BufferData!
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
			x_des_.translation() += (force_h.head(3) / intentEst_M) * intentEst_delT * intentEst_delT;
			//X_hat.head(3) = x_des_.translation();
//
//				x_des_.translation() = task_ref;
//
//				intent_elapsed_ = robot_state_->getTime() ;
//			}
		}
		if(computeHumanIntentNN && loop_count_ > 10 ) //&& loop_count_ % 5 == 0) // TODO make sure this is executed before BufferData!
		{
			ptrNNEstimator->Update(X,Xd,force_h,dt_,X_hat, Xd_hat);		// TODO use correct dt
		}
		/*
		if(calibrateSensors)
		{

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
					// Step function (global frame)
//					x_r.topRows(3) = x0_cali_vec_.topRows(3) + maxCalibrationDistance_*sensorDirections.col(tactileSensorSelected_);
//					x_d.topRows(3) = x0_cali_vec_.topRows(3);

					// Step function (gripper frame)
					x_r.topRows(3) = maxCalibrationDistance_*sensorDirections.col(tactileSensorSelected_);
					x_d.topRows(3).setZero();

					// Initialize model trajectory
					X_m = x0_cali_vec_;

					// Start data recording
					experiment_ = PR2adaptNeuroControllerClass::B;
					storage_index_ = 0;
					recordData = true;

					// Set counter and flag
					calibrationCounter = 0;
					refTrajSetForCalibration = true;
				}
				// Update distance moved
				if(refTrajSetForCalibration)
				{
					calibrationDistance_ = ( x_.translation() - x0_cali_.translation() ).norm();
				}
			}

			// Check if x_r has been reached
			if(calibrationDistance_ > maxCalibrationDistance_ -0.01)	// TODO better threshold
			{
				calibrationCounter++;
			}
			if( calibrationCounter>167)		// Wait 0.5 seconds extra
			{
				calibrationCounter = 0;
				calibrateSensors = false;
				refTrajSetForCalibration = false;
				recordData = false;

				// Fix filter weights
				for(int i=0; i<numTactileSensors_;i++)
				{
					ARMAmodel_flexi_[i]->setUseFixedWeights(true);
				}
			}
		}

		if(experiment_ == PR2adaptNeuroControllerClass::D)
		{
			// Check if it is time to update desired position
			if(traj_msgs_.request.header.stamp < ros::Time::now())
			{
				int index = traj_msgs_.request.header.seq;

				// Check if all poses have been reached
				if(index < traj_msgs_.request.x.size())
				{
					if(dim == PR2adaptNeuroControllerClass::Pose)
					{
						PoseVec tmp;
						tmp(0) = traj_msgs_.request.x[index].position.x;
						tmp(1) = traj_msgs_.request.x[index].position.y;
						tmp(2) = traj_msgs_.request.x[index].position.z;
						tmp(3) = traj_msgs_.request.x[index].orientation.x;
						tmp(4) = traj_msgs_.request.x[index].orientation.y;
						tmp(5) = traj_msgs_.request.x[index].orientation.z;
						tmp(6) = traj_msgs_.request.x[index].orientation.w;

						std::cout<<"ExpD pose #"<<index<<": "<<tmp.transpose()<<"\n";

						x_des_ = PoseVec2Affine(tmp);
					}
					else
					{
						CartVec tmp;
						tmp(0) = traj_msgs_.request.x[index].position.x;
						tmp(1) = traj_msgs_.request.x[index].position.y;
						tmp(2) = traj_msgs_.request.x[index].position.z;

						Eigen::Quaterniond q( traj_msgs_.request.x[index].orientation.w,
								traj_msgs_.request.x[index].orientation.x,
								traj_msgs_.request.x[index].orientation.y,
								traj_msgs_.request.x[index].orientation.z);

						tmp.bottomRows(3) = quaternion2Euler( q );

						std::cout<<"ExpD pose #"<<index<<": "<<tmp.transpose()<<"\n";

						x_des_ = CartVec2Affine(tmp);
					}

					traj_msgs_.request.header.stamp = ros::Time::now() + ros::Duration(traj_msgs_.request.t[index]);
					traj_msgs_.request.header.seq++;
				}
				else
				{
					experiment_ = PR2adaptNeuroControllerClass::Done;
				}
			}
		}
		*/

		if(externalRefTraj)
		{
			// Subscriber updates commandPose, commandTf
			// TODO: interpolate?, set xd_des_
			x_des_ = commandPose;
		}

		if( (useARMAmodel || tuneARMA ) && loop_count_ > 10)
		{
			for(int i=0;i<2;i++)
			{
				double xm, xdm, xddm;
				double xd = X(i);	// TODO or use X_m(i) ?

				ARMAmodel_FT_[i]->runARMAupdate(dt_       ,  // input: delta T
                                                force_h(i),  // input:  force or voltage
                                                xd        ,  // input:  x_d
                                                xm        ,  // output: x_m
                                                xdm       ,  // output: xd_m
                                                xddm     );  // output: xdd_m

				if(tuneARMA)
				{
					Eigen::MatrixXd tmp;
					ARMAmodel_FT_[i]->getWeights(tmp);
					weightsARMA_FT_.col(i) = tmp;
				}

				X_hat(i)  = xm;
				Xd_hat(i) = xdm;
			}
			x_des_  = CartVec2Affine(X_hat);
		}

		// Update reference trajectory vectors for NN
		if(useHumanIntentNN && loop_count_ > 10)
		{
			// Assume nne_Dim <= 3, i.e. only position
			e_int = X_hat.head(3) - X.head(3);

			// Limit error
			if(nne_useLimits_err)
			{
				e_int = (e_int.array() > e_int_max.array() ).select(e_int_max, e_int);
				e_int = (e_int.array() < e_int_min.array() ).select(e_int_min, e_int);
			}

			/*
			if(loop_count_ < 5000)	// Allow X_hat some time to converge
			{
				X_hat.head(3) = x0_vec_.head(3) + nne_scaling * e_int;
				nne_scaling += 0.0002;
			}
			*/

			// Lowpass filter position prediction
			X_m.head(3) = X.head(3) + nne_pose_filter * e_int;	// Note: X_m = X_hat if nne_pose_filter = 1.0
			X_m.tail(3) = Eigen::Vector3d::Zero();

			x_des_  = CartVec2Affine(X_m);
			//xd_des_ = CartVec2Affine(Xd_hat);

			//convert2NNinput(X_hat, X_m);
			//convert2NNinput(Xd_hat, Xd_m);
		}
		else
		{
			convert2NNinput(x_des_, X_m);
			//convert2NNinput(xd_des_, Xd_m);
			//convert2NNinput(xdd_des_, Xdd_m);
		}

		// Calculate Cartesian error
		computePoseError(x_des_,x_, xerr_);				// e = xd - x, TODO: Use xd_filtered_ instead
		X_m.tail(3) = X.tail(3) + xerr_.tail(3);		// Make sure X_m - X rotation error is small/continuous

		// TODO
		/*
		X_m(3) = 1.57; Xd_m(3) = 0.0; Xdd_m(3) = 0.0;
		X_m(4) = 0.0;  Xd_m(4) = 0.0; Xdd_m(4) = 0.0;
		X_m(5) = 0.0;  Xd_m(5) = 0.0; Xdd_m(5) = 0.0;
		*/

		/***************** FEEDFORWARD FORCE *****************/

//		if(useFlexiForce)
//		{
//			tau_ = JacobianTrans*(-fFForce*tactile_wrench_);	// [7x6]*[6x1]->[7x1]
//		}

//		if(forceTorqueOn)
//		{
//			tau_ = JacobianTrans*(fFForce*wrench_filtered_);	// [7x6]*[6x1]->[7x1]
//		}

		/***************** OUTER LOOP *****************/
/*
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

			if(calibrateSensors  && refTrajSetForCalibration)	// TODO: check if tactileSensorSelected_ within range
			{
				Eigen::VectorXd tmp;
				tmp.resize(4);
				tmp.setZero();

				// Compute x_d
				prev_x_d = x_d;
				x_d.topRows(3) = ( task_mA*outer_delT*x_r.topRows(3) + prev_x_d.topRows(3) ) / ( 1+task_mB*outer_delT );

				// Project into sensor axis to get x_d > 0
				tmp(0) = x_d.topRows(3).dot( sensorDirections.col(tactileSensorSelected_) );
				tmp(0) = fabs( tmp(0) );	// FIXME make sure it's >0

				// Update ARMA model and get x_m
				ARMAmodel_flexi_[tactileSensorSelected_]->runARMAupdate(outer_delT  ,  // input: delta T
							tactile_data_(tactileSensorSelected_)        			,  // input:  force or voltage
							tmp(0)      											,  // input:  x_d
							tmp(1)      											,  // output: x_m
							tmp(2)     												,  // output: xd_m
							tmp(3)    												);  // output: xdd_m
//std::cout<<"ARMA:\n"<<tmp<<"\n---\n";
				// Convert into global reference frame
				X_m.topRows(3) = x0_cali_vec_.topRows(3).cwiseProduct(Vec3d_ones - sensorDirections.col(tactileSensorSelected_).cwiseAbs());
				X_m.topRows(3) = x0_cali_vec_.topRows(3)+tmp(1)*sensorDirections.col(tactileSensorSelected_);
//				X_m.topRows(3) = X.topRows(3)+tmp(1)*sensorDirections.col(tactileSensorSelected_);	// <- Results in overshoot

				// Save weights
				for(int i=0; i<numTactileSensors_;i++)
				{
					Eigen::MatrixXd tmp2;
					ARMAmodel_flexi_[i]->getWeights(tmp2);
					filterWeights_flexi_.col(i) = tmp2;
				}

			}
			else if(tactileSensorSelected_==-1)		// Use fixed weights
			{
				Eigen::VectorXd tmp;
				tmp.resize(4);
				tmp.setZero();

				X_m.topRows(2) = X.topRows(2);	// z, orientation are fixed <- works lika a mannequin mode!

				for(int i=0;i<numTactileSensors_;i++)
				{

					ARMAmodel_flexi_[i]->updateDelT(outer_delT);
					ARMAmodel_flexi_[i]->useARMA( tmp(0),				// output: x_m
												  tmp(1),				// output: xd_m
												  tmp(2),				// output: xdd_m
												  tactile_data_(i) );	// input:  force or voltage
//std::cout<<"ARMA:\n"<<tmp(0)<<"\n---\n";
					X_m.topRows(3) += tmp(0)*sensorDirections.col(i);
				}
				//std::cout<<"X_m:\n"<<X_m<<"\n---\n";
			}

			outer_elapsed_ = robot_state_->getTime() ;

			// Output variables:
			//   x_d;
			//   X_m;
			//   Xd_m;
			//   Xdd_m;

		}
*/
		/***************** INNER LOOP *****************/

		force_c_prev = force_c;

		// Neural Network
		ptrNNController->UpdateCart(q, qd, X, Xd, X_m, Xd_m, Xdd_m, dt_,force_h,force_c);		// TODO check sign of force_h

		// Check for valid output
		for(int i=0;i<force_c.size();i++)
		{
			if( isnan(force_c(i)) )
			{
				//std::cout<<"force_c("<<i<<") is NaN: "<<force_c.transpose()<<"\n";
				force_c(i) = force_c_prev(i);
				invalid_value_count_ += 100;
			}
			if( isinf(force_c(i)) )
			{
				//std::cout<<"force_c("<<i<<") is Inf: "<<force_c.transpose()<<"\n";
				force_c(i) = force_c_prev(i);
				invalid_value_count_ += 1000;
			}
		}

		// Convert NN result to a Cartesian vector
		Force6d.setZero();
		convert2CartVec(force_c, Force6d);

//		if(fFForce)
//		{
//			force_c -= t_r;
//		}

		// PD controller
/*
		CartVec kp, kd;
		kp << 100.0,100.0,100.0,100.0,100.0,100.0;
		kd << 1.0,1.0,1.0,1.0,1.0,1.0;
		// F    = -(       Kp * (x-x_dis)   +     Kd * (xdot - 0)    )
		force_c = -(kp.asDiagonal() * xerr_ + kd.asDiagonal() * xdot_);			// TODO choose NN/PD with a param
*/
		// JT Cartesian
/*
		Eigen::Vector3d px(x_.translation());
		Eigen::Vector3d pdes(x_des_.translation());
		px.head(2) = pdes.head(2);
		Eigen::Affine3d t0 = x_;
		t0.translation() = px;		// Make xy error zero

		Eigen::VectorXd t1 = xdot_;

		ptrJTController->update(t0,t1,x_des_,fc_JT_);
		//std::cout<<"fc: "<<fc_JT_.transpose()<<"\n";
		Force6d = fc_JT_;
*/

		tau_ = tau_ + JacobianTrans*Force6d;		// [7x6]*[6x1]->[7x1]

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

			/*
			for (int j = 0; j < Joints; ++j)
			{
				// This is the Liegeois cost function from 1977
				q_jointLimit(j) = - (q(j) - qnom(j) )/( q.rows() * ( q_upper(j) - q_lower(j))) ;
			}
			nullspaceTorque = nullSpace*50*( q_jointLimit - 0.0*qd );
			*/

			// Posture control from JT Cartesian controller:
			JointVec posture_err = q_posture_ - q;
			for (size_t j = 0; j < Joints; ++j)
			{
				if (chain_.getJoint(j)->joint_->type == urdf::Joint::CONTINUOUS)
					posture_err[j] = angles::normalize_angle(posture_err[j]);
			}

			for (size_t j = 0; j < Joints; ++j) {
				if (fabs(q_posture_[j] - 9999) < 1e-5)
					posture_err[j] = 0.0;
			}
			JointVec qdd_posture = k_posture * posture_err;
			nullspaceTorque = joint_dd_ff_.array() * (nullSpace * qdd_posture).array();

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

/* wrap x -> [0,max) */
double wrapMax(double x, double max)
{
    return fmod(max + fmod(x, max), max);
}
/* wrap x -> [min,max) */
double wrapMinMax(double x, double min, double max)
{
    return min + wrapMax(x - min, max - min);
}

/// Controller update loop in realtime
void PR2adaptNeuroControllerClass::update()
{

	++loop_count_;	// Start at 1

	/***************** GET ROBOT STATE *****************/

	if(loop_count_ % loopRateFactor == 1 && !runComputations)	// Retrieve data and start computations in a thread
	{
		// Calculate the dt between servo cycles.
		dt_ = (robot_state_->getTime() - last_time_).toSec();
		last_time_ = robot_state_->getTime();

		// Get the current joint positions and velocities.
		chain_.getPositions(q_);
		chain_.getVelocities(qdot_);

		// Wrap continuous joints
		//q_(4) = wrapMinMax(q_(4), -M_PI, +M_PI);	<-- Might cause discontinuities
		//q_(6) = wrapMinMax(q_(6), -M_PI, +M_PI);

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
		//chain_.getVelocities(qdot_raw_);
		for (int i = 0; i < Joints; ++i)
			qdot_filtered_[i] += joint_vel_filter_ * (qdot_[i] - qdot_filtered_[i]);	// Does nothing when joint_vel_filter_=1
		qdot_ = qdot_filtered_;

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
			// TODO check if CB function has been updated
		}

		if( accelerometerOn )//|| forceTorqueOn )
		{
			// Retrieve accelerometer data
			accData_vector.clear();
			accData_vector = accelerometer_handle_->state_.samples_;
			accData_vector_size = accData_vector.size();			// 3 or 4 (usually three)

			if(accData_vector_size > 0)
			{
				accData_received = true;

				// Convert into Eigen vector and compute average value
				accData.setZero();
				for( int  i = 0; i < accData_vector_size; i++ )		// Take average value TODO use median?
				{
					accData(0) += accData_vector[i].x;
					accData(1) += accData_vector[i].y;
					accData(2) += accData_vector[i].z;
				}
				accData = accData.array() / (double)accData_vector_size;
			}
			else
			{
				// Use previous value
				accData_received = false;
				missed_updates_count_ += 100;
			}

		}

		if( forceTorqueOn )
		{
			// Retrieve Force/Torque data
			ftData_vector.clear();
			ftData_vector = ft_handle_->state_.samples_;
			ftData_vector_size = ftData_vector.size();				// between 0 and 4 (usually three)

			if(ftData_vector_size > 0)
			{
				ftData_received = true;

				// Convert into Eigen vector and compute average
				wrenchData.setZero();
				for( int  i = 0; i < ftData_vector_size; i++ )		// Take average value TODO use median?
				{
					tf::wrenchMsgToEigen(ftData_vector[i], wrench_raw_);
					wrenchData += wrench_raw_;
				}
				wrenchData = wrenchData.array() / (double)ftData_vector_size;
			}
			else
			{
				// Use previous value
				ftData_received = false;
				missed_updates_count_ += 10000;
			}
		}

		/***************** START DATA PROCESSING *****************/
		runComputations = true;
		m_Condition.notify_one();
	}

	/***************** UPDATE TORQUE COMMAND  *****************/

	if(loop_count_ % loopRateFactor == 0)	// After X loops, assume computations are done and send commands
	{
		if(runComputations)
		{
			missed_updates_count_ += 1;		// computations still running
		}
//		else
//		{
//			tau_c_latest_ = tau_c_; 	// computations finished, update torque command value
//		}
	}
	if(!runComputations)
	{
		tau_c_latest_ = tau_c_; 	// computations finished, update torque command value
	}

	/***************** SEND TORQUE COMMAND  *****************/
	//if(loop_count_ > loopRateFactor)
	{
		chain_.setEfforts( tau_c_latest_ );
	}

	/***************** DATA COLLECTION *****************/
	if (recordData && loop_count_>200 && loop_count_ % 20 == 0)	// 1000Hz / 20 = 50 Hz TODO check runComputations
	{
		bufferData();
	}

	/***************** DATA PUBLISHING *****************/

	if (publishRTtopics && !runComputations )	// TODO make sure dimensions/type agree
	{
/*
		//cartvec_tmp(1) = accData_vector_size;
		//cartvec_tmp(2) = ftData_vector_size;
		cartvec_tmp(1) = nnController.getOuterWeightsNorm();
		cartvec_tmp(2) = nnController.getInnerWeightsNorm();
		//cartvec_tmp(1) = calibrationDistance_;
		//cartvec_tmp(2) = filterWeights_flexi_.norm();

		if (pub_x_desi_.trylock()) {
			pub_x_desi_.msg_.header.stamp = last_time_;
			tf::poseEigenToMsg(CartVec2Affine(cartvec_tmp), pub_x_desi_.msg_.pose);	// cartvec_tmp
			//tf::poseEigenToMsg(x_acc_to_ft_, pub_x_desi_.msg_.pose);
			pub_x_desi_.msg_.header.frame_id = "l_gripper_motor_accelerometer_link";
			pub_x_desi_.unlockAndPublish();
		}
*/
/*
		if (pub_state_.trylock()) {
			// Headers
			pub_state_.msg_.header.stamp                 = last_time_;
			pub_state_.msg_.x.header.stamp               = last_time_;
			pub_state_.msg_.x_desi_filtered.header.stamp = last_time_;
			pub_state_.msg_.x_desi.header.stamp          = last_time_;

			pub_state_.msg_.missed_updates_counter = missed_updates_count_;

			// Pose
			tf::poseEigenToMsg(x_, pub_state_.msg_.x.pose);
			tf::poseEigenToMsg(convert2Affine(X_m), pub_state_.msg_.x_desi.pose);		// X_m, xd_T
			// tf::poseEigenToMsg(x_desi_filtered_, pub_state_.msg_.x_desi_filtered.pose);

			// Error
			tf::twistEigenToMsg(xerr_, pub_state_.msg_.x_err);

			// Twist
			tf::twistEigenToMsg(xdot_, pub_state_.msg_.xd);
			// tf::twistEigenToMsg(Xd_m, pub_state_.msg_.xd_desi);

			// Force
			if(forceTorqueOn)
			{
				tf::wrenchEigenToMsg(wrench_transformed_, pub_state_.msg_.force_measured);	// TODO use wrench_filtered?
			}
			tf::wrenchEigenToMsg(Force6d, pub_state_.msg_.force_c);			// NN control force

			//tf::matrixEigenToMsg(J_, pub_state_.msg_.J);			// TODO move into a separate message
			//tf::matrixEigenToMsg(nullSpace, pub_state_.msg_.N);

			//tf::matrixEigenToMsg(ptrNNController->getInnerWeights(), pub_state_.msg_.V);
			//tf::matrixEigenToMsg(ptrNNController->getOuterWeights(), pub_state_.msg_.W);

			for (int j = 0; j < num_Joints; ++j) {
				pub_state_.msg_.tau_c[j] = tau_c_latest_(j);
				pub_state_.msg_.q[j] = q_(j);
				if(useNullspacePose)
					pub_state_.msg_.tau_posture[j] = nullspaceTorque(j);
			}

			pub_state_.msg_.W_norm = ptrNNController->getInnerWeightsNorm();
			pub_state_.msg_.V_norm = ptrNNController->getOuterWeightsNorm();

			pub_state_.unlockAndPublish();
		}

		if (pub_ft_.trylock()) {
			pub_ft_.msg_.header.stamp = last_time_;
			//pub_ft_.msg_.wrench = l_ftData.wrench;
			pub_ft_.unlockAndPublish();
		}
*/

		if (pub_ft_transformed_.trylock()) {
			pub_ft_transformed_.msg_.header.stamp = last_time_;
			if(useDigitalFilter)
				tf::wrenchEigenToMsg(wrench_filtered_, pub_ft_transformed_.msg_.wrench);
			else
				tf::wrenchEigenToMsg(wrench_transformed_, pub_ft_transformed_.msg_.wrench);
			pub_ft_transformed_.unlockAndPublish();
		}

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
		}

		// CT RLS ARMA
		// RLS FIR
		// MRAC
		// MSD
		// IRL

		// Direct Model
		if( useDirectmodel )
		{
			// q_d
			X_m(1)= task_refModel_output(1)      ;
			Xd_m  = (X_m - p_X_m)/outerLoopTime  ;
			Xdd_m = (Xd_m - p_Xd_m)/outerLoopTime;
		}
		// MRAC

		p_X_m    = X_m   ;
		p_Xd_m   = Xd_m  ;
		p_Xdd_m  = Xdd_m ;

		outer_elapsed_ = robot_state_->getTime() ;

	}

	// System Model END
	/////////////////////////
}

void PR2adaptNeuroControllerClass::commandPoseCB(const geometry_msgs::PoseStamped::ConstPtr &p)
{
	// TODO perform tf transform, assume torso_lift_link for now
	// if(p->header.frame_id == "torso_lift_link")
	commandTf = p->header.stamp;

	tf::poseMsgToEigen(p->pose, commandPose);
}


void PR2adaptNeuroControllerClass::bufferData()
{
	if( (storage_index_ < 0) || (storage_index_ >= StoreLen) )
	{
		std::cout<<"No data is being recorded!\n";
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
		experimentDataA_msg_[storage_index_].Wnorm = ptrNNController->getOuterWeightsNorm();
		experimentDataA_msg_[storage_index_].Vnorm = ptrNNController->getInnerWeightsNorm();

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
		experimentDataB_msg_[storage_index_].Wnorm = ptrNNController->getOuterWeightsNorm();
		experimentDataB_msg_[storage_index_].Vnorm = ptrNNController->getInnerWeightsNorm();

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
		experimentDataB_msg_[storage_index_].x_i.x     = x0_cali_vec_(0) + x_r(0);
		experimentDataB_msg_[storage_index_].x_i.y     = x0_cali_vec_(1) + x_r(1);
		experimentDataB_msg_[storage_index_].x_i.z     = x0_cali_vec_(2) + x_r(2);
		experimentDataB_msg_[storage_index_].x_i.phi   = x0_cali_vec_(3) + x_r(3);
		experimentDataB_msg_[storage_index_].x_i.the   = x0_cali_vec_(4) + x_r(4);
		experimentDataB_msg_[storage_index_].x_i.psi   = x0_cali_vec_(5) + x_r(5);

		// Task trajectory
		experimentDataB_msg_[storage_index_].x_d.x     = x0_cali_vec_(0) + x_d(0);
		experimentDataB_msg_[storage_index_].x_d.y     = x0_cali_vec_(1) + x_d(1);
		experimentDataB_msg_[storage_index_].x_d.z     = x0_cali_vec_(2) + x_d(2);  // = 0
		experimentDataB_msg_[storage_index_].x_d.phi   = x0_cali_vec_(3) + x_d(3);  // = 0
		experimentDataB_msg_[storage_index_].x_d.the   = x0_cali_vec_(4) + x_d(4);  // = 0
		experimentDataB_msg_[storage_index_].x_d.psi   = x0_cali_vec_(5) + x_d(5);  // = 0

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
		//experimentDataC_msg_[storage_index_].RBFnorm           = rbfnnController.getRBFNorm();
		//experimentDataC_msg_[storage_index_].TANHnorm          = rbfnnController.getTANHNorm();

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
		experimentDataC_msg_[storage_index_].tau.j0   = tau_c_latest_(0);
		experimentDataC_msg_[storage_index_].tau.j1   = tau_c_latest_(1);
		experimentDataC_msg_[storage_index_].tau.j2   = tau_c_latest_(2);
		experimentDataC_msg_[storage_index_].tau.j3   = tau_c_latest_(3);
		experimentDataC_msg_[storage_index_].tau.j4   = tau_c_latest_(4);
		experimentDataC_msg_[storage_index_].tau.j5   = tau_c_latest_(5);
		experimentDataC_msg_[storage_index_].tau.j6   = tau_c_latest_(6);

		experimentDataC_msg_[storage_index_].tau_sat.j0   = tau_sat(0);
		experimentDataC_msg_[storage_index_].tau_sat.j1   = tau_sat(1);
		experimentDataC_msg_[storage_index_].tau_sat.j2   = tau_sat(2);
		experimentDataC_msg_[storage_index_].tau_sat.j3   = tau_sat(3);
		experimentDataC_msg_[storage_index_].tau_sat.j4   = tau_sat(4);
		experimentDataC_msg_[storage_index_].tau_sat.j5   = tau_sat(5);
		experimentDataC_msg_[storage_index_].tau_sat.j6   = tau_sat(6);
	}
	else if (experiment_ == PR2adaptNeuroControllerClass::NACwithHIE)
	{
		//StateMsg* pMsg = &experimentDataState_msg_[storage_index_];

		experimentDataState_msg_[storage_index_].header.stamp = last_time_;
		experimentDataState_msg_[storage_index_].dt = dt_;
		experimentDataState_msg_[storage_index_].missed_updates_counter = missed_updates_count_;
		missed_updates_count_ = 0;

		experimentDataState_msg_[storage_index_].invalid_value_counter = invalid_value_count_;
		invalid_value_count_ = 0;

		// Pose
		tf::poseEigenToMsg(x_, experimentDataState_msg_[storage_index_].x.pose);
		tf::twistEigenToMsg(xdot_, experimentDataState_msg_[storage_index_].xd);

		// Force
		if(forceTorqueOn)
		{
			if(useDigitalFilter)
				tf::wrenchEigenToMsg(wrench_filtered_, experimentDataState_msg_[storage_index_].force_measured);	// TODO save both
			else
				tf::wrenchEigenToMsg(wrench_transformed_, experimentDataState_msg_[storage_index_].force_measured);
			tf::wrenchEigenToMsg(force_h, experimentDataState_msg_[storage_index_].force_h);
		}
		//tf::matrixEigenToMsg(J_, pub_state_.msg_.J);			// TODO move into a separate message
		//tf::matrixEigenToMsg(nullSpace, pub_state_.msg_.N);

		// Reference trajectory
		tf::poseEigenToMsg(convert2Affine(X_m), experimentDataState_msg_[storage_index_].x_desi.pose);		// X_m, xd_T
		// tf::poseEigenToMsg(x_desi_filtered_, experimentDataState_msg_[storage_index_].x_desi_filtered.pose);
		tf::twistEigenToMsg(Xd_m,  experimentDataState_msg_[storage_index_].xd_desi);
		tf::twistEigenToMsg(xerr_, experimentDataState_msg_[storage_index_].x_err);

		// Controller
		tf::wrenchEigenToMsg(Force6d, experimentDataState_msg_[storage_index_].force_c);			// NN control force
		experimentDataState_msg_[storage_index_].W_norm = ptrNNController->getInnerWeightsNorm();
		experimentDataState_msg_[storage_index_].V_norm = ptrNNController->getOuterWeightsNorm();
		//tf::matrixEigenToMsg(ptrNNController->getInnerWeights(), pub_state_.msg_.V);
		//tf::matrixEigenToMsg(ptrNNController->getOuterWeights(), pub_state_.msg_.W);

		// Vectors
		experimentDataState_msg_[storage_index_].q.resize(num_Joints);
		experimentDataState_msg_[storage_index_].tau_c.resize(num_Joints);
		if(useNullspacePose)
			experimentDataState_msg_[storage_index_].tau_posture.resize(num_Joints);

		for (int j = 0; j < num_Joints; ++j) {
			experimentDataState_msg_[storage_index_].tau_c[j] = tau_c_latest_(j);
			experimentDataState_msg_[storage_index_].q[j] = q_(j);
			if(useNullspacePose)
				experimentDataState_msg_[storage_index_].tau_posture[j] = nullspaceTorque(j);
		}

		// ARMA model
		if(useARMAmodel || tuneARMA)
		{
			tf::poseEigenToMsg(convert2Affine(X_hat), experimentDataState_msg_[storage_index_].x_hat);
			tf::poseEigenToMsg(convert2Affine(Xd_hat), experimentDataState_msg_[storage_index_].xd_hat);
		}
		if(tuneARMA)
		{
			experimentDataState_msg_[storage_index_].armaX.resize(8);
			experimentDataState_msg_[storage_index_].armaY.resize(8);
			for (int j = 0; j < 8; ++j) {
				experimentDataState_msg_[storage_index_].armaX[j] = weightsARMA_FT_(j,0);
				experimentDataState_msg_[storage_index_].armaY[j] = weightsARMA_FT_(j,1);
			}
		}

		// Estimator
		if(computeHumanIntentNN)
		{
			tf::poseEigenToMsg(convert2Affine(X_hat), experimentDataState_msg_[storage_index_].x_hat);
			tf::poseEigenToMsg(convert2Affine(Xd_hat), experimentDataState_msg_[storage_index_].xd_hat);
			experimentDataState_msg_[storage_index_].U_norm_traj = ptrNNEstimator->getWeightsNormU();
			experimentDataState_msg_[storage_index_].V_norm_gain = ptrNNEstimator->getWeightsNormV();

			Kh = ptrNNEstimator->getKh();
			Dh = ptrNNEstimator->getDh();
			nne_s = ptrNNEstimator->getS();
			nne_ea = ptrNNEstimator->getEa();

			experimentDataState_msg_[storage_index_].Kh.resize(nne_Dim);
			experimentDataState_msg_[storage_index_].Dh.resize(nne_Dim);
			experimentDataState_msg_[storage_index_].nne_s.resize(nne_Dim);
			experimentDataState_msg_[storage_index_].nne_ea.resize(nne_Dim);

			for (int j = 0; j < nne_Dim; ++j) {
				experimentDataState_msg_[storage_index_].Kh[j] = Kh(j);
				experimentDataState_msg_[storage_index_].Dh[j] = Dh(j);
				experimentDataState_msg_[storage_index_].nne_s[j] = nne_s(j);
				experimentDataState_msg_[storage_index_].nne_ea[j] = nne_ea(j);
			}

		}

		// Prescribed error dynamics
		if(nn_usePED)
		{
			Gamma  = ptrNNController->getGa();
			Lambda = ptrNNController->getLa();

			experimentDataState_msg_[storage_index_].Gamma.resize(Gamma.size());
			for (int j = 0; j < Gamma.size(); ++j) {
				experimentDataState_msg_[storage_index_].Gamma[j] = Gamma(j);
			}
			experimentDataState_msg_[storage_index_].Lambda.resize(Lambda.size());
			for (int j = 0; j < Lambda.size(); ++j) {
				experimentDataState_msg_[storage_index_].Lambda[j] = Lambda(j);
			}
		}
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

	kappa       = req.msg.kappa                    ;
//	Kv          = req.msg.Kv                       ;
//	lambda      = req.msg.lambda                   ;
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


	useCurrentCartPose= req.msg.useCurrentCartPose ;
	useNullspacePose  = req.msg.useNullspacePose   ;

	cartIniX          = req.msg.cartIniX           ;
	cartIniY          = req.msg.cartIniY           ;
	cartIniZ          = req.msg.cartIniZ           ;
	cartIniRoll       = req.msg.cartIniRoll        ;
	cartIniPitch      = req.msg.cartIniPitch       ;
	cartIniYaw        = req.msg.cartIniYaw         ;

	// Update NN
	delete ptrNNController;

	if( !initNN() )
	{
		ROS_ERROR("initNN() failed!");
	}

	resp.success = true;

	return true;
}

/// Service call to publish the saved data
bool PR2adaptNeuroControllerClass::publishExperimentData( std_srvs::Empty::Request & req,
                                                std_srvs::Empty::Response& resp )
{
	recordData = false;

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
		if(	experiment_ == PR2adaptNeuroControllerClass::NACwithHIE)
			pubExperimentDataState_.publish(experimentDataState_msg_[index]);

	}

	return true;
}

bool PR2adaptNeuroControllerClass::setNNparamCB( ice_msgs::setParameters::Request & req,
											 ice_msgs::setParameters::Response& resp )
{
	if( req.names.size() != req.values.size() )
	{
		resp.success = false;
	}
	else
	{
		resp.success = true;

		// Grab the resource or wait until free
		//boost::mutex::scoped_lock lock(m_Mutex);

		for(int i=0;i<req.names.size();i++)
		{
			if( req.names[i].compare("Kz") ) {
				ptrNNController->setParamKz(req.values[i]);
			}
			else if ( req.names[i].compare("Zb") ) {
				ptrNNController->setParamZb(req.values[i]);
			}
			else if ( req.names[i].compare("kappa") ) {
				ptrNNController->setParamKappa(req.values[i]);
			}
			else if ( req.names[i].compare("F") ) {
				ptrNNController->setParamF(req.values[i]);
			}
			else if ( req.names[i].compare("G") ) {
				ptrNNController->setParamG(req.values[i]);
			}
			else if ( req.names[i].substr(0,2).compare("Kv") )
			{
				try {
					int idx = boost::lexical_cast<int>( req.names[i].substr(2,1) );
					ptrNNController->setParamKv(req.values[i],idx);

				} catch( boost::bad_lexical_cast const& ) {
					std::cout << "Error: input string was not valid" << std::endl;
					resp.success = false;
				}
			}
			else if ( req.names[i].substr(0,2).compare("La") )
			{
				try {
					int idx = boost::lexical_cast<int>( req.names[i].substr(2,1) );
					ptrNNController->setParamLa(req.values[i],idx);

				} catch( boost::bad_lexical_cast const& ) {
					std::cout << "Error: input string was not valid" << std::endl;
					resp.success = false;
				}
			}
			else if ( req.names[i].substr(0,2).compare("Kd") )
			{
				try {
					int idx = boost::lexical_cast<int>( req.names[i].substr(2,1) );
					ptrNNController->setParamKd(req.values[i],idx);

				} catch( boost::bad_lexical_cast const& ) {
					std::cout << "Error: input string was not valid" << std::endl;
					resp.success = false;
				}
			}
			else if ( req.names[i].substr(0,2).compare("Dd") )
			{
				try {
					int idx = boost::lexical_cast<int>( req.names[i].substr(2,1) );
					ptrNNController->setParamDd(req.values[i],idx);

				} catch( boost::bad_lexical_cast const& ) {
					std::cout << "Error: input string was not valid" << std::endl;
					resp.success = false;
				}
			}
			else if ( req.names[i].compare("k_posture") ) {
				k_posture = req.values[i];
			}
			else if ( req.names[i].compare("ft_filter") ) {
				ft_filter_ = req.values[i];
			}
			else if ( req.names[i].compare("joint_vel_filter") ) {
				joint_vel_filter_ = req.values[i];
			}
			else if ( req.names[i].compare("nne_pose_filter") ) {
				nne_pose_filter = req.values[i];
			}
			else if ( req.names[i].compare("nne_alpha") ) {
				ptrNNEstimator->setParamAlpha(req.values[i]);
			}
			else if ( req.names[i].compare("nne_kappa") ) {
				ptrNNEstimator->setParamKappa(req.values[i]);
			}
			else
			{
				resp.success = false;
			}
		}
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
			Eigen::MatrixXd tmp2;
			ARMAmodel_flexi_[i]->getWeights(tmp2);
			filterWeights_flexi_.col(i) = tmp2;
			ARMAmodel_flexi_[i]->setFirstTime(true);
		}

		X_m   = affine2CartVec(x0_);
		Xd_m.setZero();
		Xdd_m.setZero();

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
			filterWeights_flexi_.col(i) = tmp;
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
	/*
	nnController.setUpdateWeights(false);
	Eigen::MatrixXd V_trans;
	Eigen::MatrixXd W_trans;
	//V_trans.setOnes( num_Hidden , num_Inputs + 1 ) ;
	V_trans.setRandom( num_Hidden , num_Inputs + 1 ) ;
	W_trans.setZero(   num_Outputs, num_Hidden     ) ;
	nnController.setInnerWeights(V_trans);
	nnController.setOuterWeights(W_trans);
	*/
	ptrNNController->setUpdateWeights(true);

	// Set circle rate and decide if the inner weights will be updated
	if(req.value > 0)
	{
		ptrNNController->setUpdateInnerWeights(true);
		circle_rate = req.value;
	}
	else
	{
		ptrNNController->setUpdateInnerWeights(false);
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

	// Reset NN
	ptrNNController->setUpdateWeights(false);
	ptrNNController->resetWeights(0.01);
	ptrNNController->setUpdateWeights(true);

	// Decide if the inner weights will be updated
	if(req.value > 0)
	{
		ptrNNController->setUpdateInnerWeights(true);
	}
	else
	{
		ptrNNController->setUpdateInnerWeights(false);
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
		//rbfnnController.resetWeights();
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

bool PR2adaptNeuroControllerClass::runExperimentD(	ice_msgs::setTrajectory::Request & req,
													ice_msgs::setTrajectory::Response& resp )
{
	experiment_ = PR2adaptNeuroControllerClass::D;

	/* Record the starting time. */
	ros::Time started = ros::Time::now();

	// Re-set NN

	// Set trajectory
	traj_msgs_.request = req;

	traj_msgs_.request.header.seq = 0;						// Use as index
	traj_msgs_.request.header.stamp = ros::Time::now()+ros::Duration(traj_msgs_.request.t[0]);	// Use time for trajectory execution

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
	experiment_ = PR2adaptNeuroControllerClass::NACwithHIE;
	storage_index_ = 0;
	loop_count_ = 0;
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

	return true;
}

bool PR2adaptNeuroControllerClass::updateInnerNNweights( ice_msgs::setBool::Request& req,
		                                            ice_msgs::setBool::Response& resp )
{

	ptrNNController->setUpdateInnerWeights(req.variable);
	resp.success = true;

	return true;
}
bool PR2adaptNeuroControllerClass::updateNNweights( ice_msgs::setBool::Request& req,
		                                            ice_msgs::setBool::Response& resp )
{

	ptrNNController->setUpdateWeights(req.variable);
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
    	Eigen::MatrixXd V_;
    	Eigen::MatrixXd W_;
		V_.resize( num_Inputs + 1, num_Hidden ) ;
		W_.resize( num_Hidden + 1, num_Outputs ) ;

		int i = 0;
		for(int r=0;r<num_Inputs + 1;r++)
		{
			for(int c=0;c<num_Hidden;c++)
			{
				V_(r,c) = req.net.V.data[i];
				i++;
			}
		}

		i=0;
		for(int r=0;r<num_Hidden+1;r++)
		{
			for(int c=0;c<num_Outputs;c++)
			{
				W_(r,c) = req.net.W.data[i];
				i++;
			}
		}

		ptrNNController->setInnerWeights(V_);
		ptrNNController->setOuterWeights(W_);

		resp.success = true;
    }
	return true;
}
bool PR2adaptNeuroControllerClass::getNNweights( ice_msgs::getNNweights::Request& req,
		                                         ice_msgs::getNNweights::Response& resp )
{
    tf::matrixEigenToMsg(ptrNNController->getInnerWeights(), resp.net.V);
    tf::matrixEigenToMsg(ptrNNController->getOuterWeights(), resp.net.W);
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
	nh_.param("/useHumanIntentNN",  useHumanIntentNN,   false);
	nh_.param("/computeHumanIntentNN",  computeHumanIntentNN,   false);

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

	for(int i=0;i<num_Joints;i++)
	{
		std::cout<<"Joint "<<i<<": ["<<q_lower(i)<<", "<<q_upper(i)<<"]\n";
	}

	// The forearm and wrist roll joints are continuous and have no actual limits
	q_upper(4) =   M_PI ;
	q_upper(6) =   M_PI ;

	q_lower(4) = - M_PI ;
	q_lower(6) = - M_PI ;

	qnom(0) = ( q_upper(0) - q_lower(0) ) / 2 ;
	qnom(1) = ( q_upper(1) - q_lower(1) ) / 2 ;
	qnom(2) = ( q_upper(2) - q_lower(2) ) / 2 ;
	qnom(3) = ( q_upper(3) - q_lower(3) ) / 2 ;
	qnom(4) = ( q_upper(4) - q_lower(4) ) / 2 ;
	qnom(5) = ( q_upper(5) - q_lower(5) ) / 2 ;
	qnom(6) = ( q_upper(6) - q_lower(6) ) / 2 ;

	// Posture control
	loadROSparam("/k_posture", k_posture, 1.0);
	loadROSparam("/jacobian_inverse_damping", jacobian_inverse_damping, 0.0);

	for (int i = 0; i < Joints; ++i)
	{
		std::string tmp = chain_.getJoint(i)->joint_->name;

		if (!nh_.getParam( "/joint_feedforward/" + tmp, joint_dd_ff_[i] )){ ROS_ERROR("Failed to load /joint_feedforward!"); joint_dd_ff_[i]=0.0; }
		if (!nh_.getParam( "/posture/" + tmp, 			q_posture_[i]   )){ ROS_ERROR("Failed to load /posture!");           q_posture_[i]=0.0; }
		if (!nh_.getParam( "/saturation/" + tmp, 		saturation_[i]  )){ ROS_ERROR("Failed to load /saturation!"); 		 saturation_[i]=0.0; }

		// TODO check
//		loadROSparam("/joint_feedforward/" + tmp, joint_dd_ff_[i], 0.0);
//		loadROSparam("/posture/" + tmp,           q_posture_[i],   0.0);
//		loadROSparam("/saturation/" + tmp,        saturation_[i],  0.0);
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

		// TODO init FT variables
		W_mat_.setZero();

		nh_.param("/useForceScaling", useForceScaling, false);
		forceScaling.setOnes();
		loadROSparamVector("/forceScaling", forceScaling);

		nh_.param("/useForceCutOff", useForceCutOff, false);
		forceCutOff.setZero();
		loadROSparamVector("/forceCutOff", forceCutOff);
	}

	// Torque Saturation
	sat_scaling = 1.0;
	tau_sat.resize( 7 );		// num_Outputs

	// Simple lowpass filters
	nh_.param("/joint_vel_filter", joint_vel_filter_, 1.0);
	nh_.param("/ft_filter", ft_filter_, 1.0);


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
	loadROSparam("/circleRate", circle_rate, 3.0);
	loadROSparam("/circleLlim", circleLlim,  0.0);
	loadROSparam("/circleUlim", circleUlim,  1.5);

	loopsCircleTraj = 0;
	circleAmpl = (circleUlim - circleLlim)/2 ;
	circle_velocity = circle_rate*circleAmpl;

	// Initial (desired) Cartesian pose
	loadROSparam("/cartIniX"    , cartIniX    , 0.7);
	loadROSparam("/cartIniY"    , cartIniY    , 0.3);
	loadROSparam("/cartIniZ"    , cartIniZ    , 0.1);
	loadROSparam("/cartIniRoll" , cartIniRoll , 0.0);
	loadROSparam("/cartIniPitch", cartIniPitch, 0.0);
	loadROSparam("/cartIniYaw"  , cartIniYaw  , 0.0);

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
		ft_bias.setZero(6);
		if(!loadROSparamVector("/bias", ft_bias))
		{
			result=false;
		}

		// Load gripper COM pose parameter
		Eigen::VectorXd tmp;
		tmp.setZero(6);
		if(!loadROSparamVector("/gripper_com_pose", tmp))
		{
			result=false;
		}
		r_gripper_com = tmp.topRows(3);


		// Load gripper mass parameter
		loadROSparam("/gripper_mass", gripper_mass, 0.0);

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

bool PR2adaptNeuroControllerClass::initOuterLoop()
{
	bool result=true;

	loadROSparam("/m_M", m_M, 1.0);
	loadROSparam("/m_S", m_S, 0.0);
	loadROSparam("/m_D", m_D, 5.0);

	loadROSparam("/task_mA", task_mA, 0.7);
	loadROSparam("/task_mB", task_mB, 0.7);

	loadROSparam("/useCurrentCartPose", useCurrentCartPose, false);
	loadROSparam("/useNullspacePose",   useNullspacePose, false);

	loadROSparam("/useARMAmodel",   useARMAmodel, false);
	loadROSparam("/useCTARMAmodel", useCTARMAmodel, false);
	loadROSparam("/useFIRmodel",    useFIRmodel, false);
	loadROSparam("/useMRACmodel",   useMRACmodel, false);
	loadROSparam("/useMSDmodel",    useMSDmodel, false);
	loadROSparam("/useIRLmodel",    useIRLmodel, false);
	loadROSparam("/useDirectmodel", useDirectmodel, false);

	loadROSparam("/tuneARMA",   tuneARMA, false);

	loadROSparam("/externalRefTraj", externalRefTraj, true);

	loadROSparam("/intentEst_time", intentLoopTime, 0.05);
	loadROSparam("/intentEst_delT", intentEst_delT, 0.1);
	loadROSparam("/intentEst_M", intentEst_M, 1.0);

	loadROSparam("/fixedFilterWeights", useFixedWeights, true);

	double rls_lambda = 0.98 ;
	double rls_sigma  = 1000 ;

	loadROSparam("/rls_lambda", rls_lambda);
	loadROSparam("/rls_sigma",  rls_sigma);

	delT = 0.001;
	loadROSparam("/outerLoop_time", outerLoopTime, 0.0);

	loadROSparam("/useSimHuman", useSimHuman, false);
	loadROSparam("/simHuman_a",  simHuman_a, 0.1);
	loadROSparam("/simHuman_b", simHuman_b, 2.6);

	/////////////////////////
	// System Model

	q       .setZero( num_Joints ) ;
	qd      .setZero( num_Joints ) ;
	qdd     .setZero( num_Joints ) ;

	q_m     .setZero( num_Joints ) ;
	qd_m    .setZero( num_Joints ) ;
	qdd_m   .setZero( num_Joints ) ;

	// desired Cartesian states
	X_m     .setZero( num_Outputs ) ;
	Xd_m    .setZero( num_Outputs ) ;
	Xdd_m   .setZero( num_Outputs ) ;

	// Prev desired Cartesian states
	p_X_m   .setZero( num_Outputs ) ;
	p_Xd_m  .setZero( num_Outputs ) ;
	p_Xdd_m .setZero( num_Outputs ) ;

	// Cartesian states
	X       .setZero( num_Outputs ) ;
	Xd      .setZero( num_Outputs ) ;

	task_ref.setZero( num_Outputs ) ;
	task_refModel_output.setZero( num_Outputs ) ;

	//tau     .setZero( num_Joints / num_Outputs ? ) ;
	force_h  .setZero( num_Outputs ) ;
	force_c  .setZero( num_Outputs ) ;

	transformed_force = Eigen::Vector3d::Zero();
	accData           = Eigen::Vector3d::Zero();

	x_d     .setZero( num_Outputs ) ;
	prev_x_d.setZero( num_Outputs ) ;

	x_r     .setZero( num_Outputs ) ;
	xd_r    .setZero( num_Outputs ) ;
	xdd_r   .setZero( num_Outputs ) ;
	prev_x_r.setZero( num_Outputs ) ;
	delta_x	= 0;

	// Initial Reference
	task_ref = X_m ;

	JacobianPinv     = Eigen::MatrixXd::Zero( num_Joints, CartDim ) ;
	JacobianTrans    = Eigen::MatrixXd::Zero( num_Joints, CartDim ) ;
	JacobianTransPinv= Eigen::MatrixXd::Zero( CartDim, num_Joints ) ;
	nullSpace        = Eigen::MatrixXd::Zero( num_Joints, num_Joints ) ;

	nullspaceTorque  = Eigen::VectorXd::Zero( num_Joints ) ;

	Force6d.setZero(CartDim);

	/////////////////////////
	// Outer Loop Init

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

	// ARMA model in XY plane
	weightsARMA_FT_.setZero(8,2);
	Eigen::MatrixXd w;
	w.resize(8,1);
	if( loadROSparamVector("/weightsARMA_X", w) )
		weightsARMA_FT_.col(0) = w;
	if( loadROSparamVector("/weightsARMA_Y", w) )
		weightsARMA_FT_.col(1) = w;

	//std::cout<<"ARMA FT weights:\n"<<weightsARMA_FT_<<"\n---\n";

	for(int i=0;i<2;i++)
	{
		csl::outer_loop::RlsModel* tmpPtr = new csl::outer_loop::RlsModel(1, 4, 4, 1);
		tmpPtr->updateDelT( outerLoopTime );
		tmpPtr->updateAB( task_mA, task_mB );
		tmpPtr->initRls( rls_lambda, rls_sigma );
		tmpPtr->setUseFixedWeights(!tuneARMA);

		w = weightsARMA_FT_.col(i);
		tmpPtr->setWeights(w);

		ARMAmodel_FT_.push_back(tmpPtr);
	}

	// System Model END
	/////////////////////////


	return result;
}

bool PR2adaptNeuroControllerClass::initInnerLoop()
{
	int n = 0;	// Number of failures

	// Default values
	num_Outputs = 6  ; // Cartesian
	num_Hidden  = 5  ;
	num_Joints  = 7  ; // 7 DOF

	n += !loadROSparam("/nnNum_Outputs", num_Outputs);
	n += !loadROSparam("/nnNum_Hidden" , num_Hidden);
	n += !loadROSparam("/nnNum_Joints" , num_Joints);

	// TODO load from ROS param
	if(num_Outputs == 1)
		dim = PR2adaptNeuroControllerClass::AxisY;
	else if (num_Outputs == 2)
		dim = PR2adaptNeuroControllerClass::PlaneXY;
	else if (num_Outputs == 3)
		dim = PR2adaptNeuroControllerClass::Position;
	else if (num_Outputs == 7)
		dim = PR2adaptNeuroControllerClass::Pose;
	else
		dim = PR2adaptNeuroControllerClass::Cart;

	// NN parameters
	n += !loadROSparam("/nn_kappa"           , kappa);
	n += !loadROSparam("/nn_Kz"              , Kz);
	n += !loadROSparam("/nn_Zb"              , Zb);
	n += !loadROSparam("/nn_feedForwardForce", fFForce);
	n += !loadROSparam("/nn_nnF"             , nnF);
	n += !loadROSparam("/nn_nnG"             , nnG);
	n += !loadROSparam("/nn_ON"              , nn_ON);

	Kv    .setZero(num_Outputs) ;
	lambda.setZero(num_Outputs) ;

	n += !loadROSparamVector("/nn_Kv", Kv);
	n += !loadROSparamVector("/nn_lambda", lambda);

	nn_usePED = false;
	nn_Kd.setZero(num_Outputs);
	nn_Dd.setZero(num_Outputs);
	n += !loadROSparam("/nn_usePED", nn_usePED);
	n += !loadROSparamVector("/nn_Kd", nn_Kd);
	n += !loadROSparamVector("/nn_Dd", nn_Dd);

	if(n==0)
		return true;

	return false;
}

bool PR2adaptNeuroControllerClass::initNN()
{
	// NN Estimator
	double nne_kappa = 0.01;
	double nne_alpha = 1.0;
	loadROSparam("/nne_kappa", nne_kappa);
	loadROSparam("/nne_alpha", nne_alpha);

	nne_Dim = 6;	// num_Outputs
	loadROSparam("/nne_Dim", nne_Dim);
	Eigen::VectorXd nne_G; nne_G.setOnes( nne_Dim*3 + 1);
	Eigen::VectorXd nne_H; nne_H.setOnes( nne_Dim*3 + 1);
	loadROSparamVector("/nne_G", nne_G);
	loadROSparamVector("/nne_H", nne_H);

	bool nne_useLimits=false;
	loadROSparam("/nne_useLimits", nne_useLimits);
	Eigen::VectorXd nne_Pmin; nne_Pmin.setZero( 2*nne_Dim);
	Eigen::VectorXd nne_Pmax; nne_Pmax.setZero( 2*nne_Dim);
	loadROSparamVector("/nne_Pmin", nne_Pmin);
	loadROSparamVector("/nne_Pmax", nne_Pmax);

	nne_useLimits_err = false;
	loadROSparam("/nne_useLimits_err", nne_useLimits_err);

	ptrNNEstimator = new csl::neural_network::NNEstimator(nne_Dim, csl::neural_network::NNEstimator::RBF);
	ptrNNEstimator->paramInit(nne_G,nne_H,nne_kappa,0.01);
	ptrNNEstimator->setParamAlpha(nne_alpha);

	ptrNNEstimator->setPhatMin(nne_Pmin);
	ptrNNEstimator->setPhatMax(nne_Pmax);
	ptrNNEstimator->setUseLimits(nne_useLimits);

	X_hat.setZero(6);	// Note: important for Update function if nne_Dim != 6
	Xd_hat.setZero(6);

	if(nne_Dim == 2)
	{
		X_hat(2) = -0.05; // FIMXE: use variable for desired z
	}

	e_int_max << 0.25, 0.25, 0.25;
	e_int_min << -0.25, -0.25, -0.25;

	nne_pose_filter = 0.01;
	loadROSparam("/nne_pose_filter", nne_pose_filter);

	// NN Controller
	ptrNNController = new csl::neural_network::NNController(num_Joints, num_Outputs, num_Hidden, nn_usePED);
	double weightsLimit = 0.01;

	ptrNNController->paramInit(Kv,lambda,kappa,Kz,Zb,nnG,nnF,weightsLimit);

	ptrNNController->setFlagPED(nn_usePED);
	ptrNNController->setParamKd(nn_Kd);
	ptrNNController->setParamDd(nn_Dd);
	ptrNNController->setFlagNN(nn_ON);
	ptrNNController->setUpdateRate(delT);
	ptrNNController->setUpdateWeights(true);
	ptrNNController->setUpdateInnerWeights(true);

	num_Inputs = ptrNNController->getNumInputs();
	//ROS_INFO_STREAM("Number of NN inputs: " << num_Inputs);

	return true;
}

bool PR2adaptNeuroControllerClass::initNullspace()
{
	IdentityCart.setIdentity();
	IdentityJoints.setIdentity();

	return true;
}

template<typename T>
bool PR2adaptNeuroControllerClass::loadROSparam(std::string name, T &variable)
{
	if(!nh_.getParam( name, variable ))
	{
		ROS_ERROR("Failed to load ROS parameter named '%s' !)", name.c_str()) ;
		return false;
	}
	//ROS_INFO_STREAM(name << " = " << variable);
	return true;
}

template<typename T>
bool PR2adaptNeuroControllerClass::loadROSparam(std::string name, T &variable, T value)
{
	if(!nh_.getParam( name, variable ))
	{
		ROS_WARN("Failed to load ROS parameter named '%s' !)", name.c_str()) ;
		variable = value;
		return false;
	}
	//ROS_INFO_STREAM(name << " = " << variable);
	return true;
}

bool PR2adaptNeuroControllerClass::loadROSparamVector(std::string name, Eigen::VectorXd &variable)
{
	std::vector<double> tmpList;

	if(!nh_.getParam( name, tmpList ))
	{
		ROS_ERROR("Failed to load ROS parameter vector named '%s' !)", name.c_str()) ;
		return false;
	}

	if( tmpList.size() != variable.size())
	{
		ROS_ERROR("Vector sizes do not match! Failed to load ROS parameter vector named '%s' !)", name.c_str()) ;
		return false;
	}

	for(int i=0; i < tmpList.size(); i++)
	{
		variable(i) = tmpList[i];
	}

	ROS_INFO_STREAM(name << " = " << variable.transpose());
	return true;
}

bool PR2adaptNeuroControllerClass::loadROSparamVector(std::string name, Eigen::MatrixXd &variable)
{
	int c = variable.cols();
	if(c != 1)
	{
		ROS_ERROR("Vector has %d columns! Failed to load ROS parameter vector named '%s' !)", c, name.c_str()) ;
		return false;
	}
	Eigen::VectorXd v;
	v.setZero(variable.rows());
	if( loadROSparamVector(name, v) )
	{
		variable = v;
		return true;
	}
	return false;
}
bool PR2adaptNeuroControllerClass::loadROSparamVector(std::string name, CartVec &variable)
{
	Eigen::MatrixXd v = variable;
	if( loadROSparamVector(name, v) )
	{
		variable = v;
		return true;
	}
	return false;
}

/////////////////////////////////////////////////////////////////////////////////////////////

bool PR2adaptNeuroControllerClass::convert2NNinput(Eigen::Affine3d in, Eigen::VectorXd &out)
{
	Eigen::VectorXd tmp;

	if(dim == PR2adaptNeuroControllerClass::Pose)
		tmp = affine2PoseVec(in);
	else
		tmp = affine2CartVec(in);

	return convert2NNinput(tmp, out);
}

bool PR2adaptNeuroControllerClass::convert2NNinput(CartVec in, Eigen::VectorXd &out)
{
	Eigen::VectorXd tmp = in;
	return convert2NNinput(tmp, out);
}

bool PR2adaptNeuroControllerClass::convert2NNinput(Eigen::VectorXd in, Eigen::VectorXd &out)
{
	switch(dim)
	{
		case PR2adaptNeuroControllerClass::AxisY:
		{
			out(0) = in(1);	// [6x1] -> [1x1]
			break;
		}
		case PR2adaptNeuroControllerClass::PlaneXY:
		{
			out(0) = in(0);	// [6x1] -> [2x1]
			out(1) = in(1);
			break;
		}
		case PR2adaptNeuroControllerClass::Position:
		{
			out(0) = in(0);	// [6x1] -> [3x1]
			out(1) = in(1);
			out(2) = in(2);
			break;
		}
		case PR2adaptNeuroControllerClass::Cart:
		{
			if(in.size() == 7)
				out = PoseVec2CartVec(in); // [7x1] -> [6x1]
			else
				out = in; // [6x1] -> [6x1]
			break;
		}
		case PR2adaptNeuroControllerClass::Pose:
		{
			if(in.size() == 6)
				out = CartVec2PoseVec(in); // [6x1] -> [7x1]
			else
				out = in; // [7x1] -> [7x1]
			break;
		}
		default:
			ROS_ERROR("Dimension not found!");
			return false;
	}
	return true;
}

bool PR2adaptNeuroControllerClass::convert2CartVec(Eigen::VectorXd in, CartVec &out)
{
	switch(dim)
	{
		case PR2adaptNeuroControllerClass::AxisY:
		{
			out(1) = in(0);	// Assuming input vector is [1x1]
			break;
		}
		case PR2adaptNeuroControllerClass::PlaneXY:
		{
			out(0) = in(0);	// [2x1]
			out(1) = in(1);
			break;
		}
		case PR2adaptNeuroControllerClass::Position:
		{
			out(0) = in(0);	// [3x1]
			out(1) = in(1);
			out(2) = in(2);
			break;
		}
		case PR2adaptNeuroControllerClass::Cart:
		{
			out = in; // [6x1]
			break;
		}
		case PR2adaptNeuroControllerClass::Pose:
		{
			out = PoseVec2CartVec(in); // [7x1]
			break;
		}
		default:
			ROS_ERROR("Dimension not found!");
			return false;
	}
	return true;
}

Eigen::Affine3d PR2adaptNeuroControllerClass::convert2Affine(Eigen::VectorXd in)
{
	CartVec tmp;
	convert2CartVec(in, tmp);

	return CartVec2Affine(tmp);
}
