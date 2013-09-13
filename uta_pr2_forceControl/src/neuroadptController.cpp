#include "uta_pr2_forceControl/neuroadptController.h"
#include <pluginlib/class_list_macros.h>
#include <tf_conversions/tf_kdl.h>

using namespace pr2_controller_ns;

using namespace std;
using namespace boost::numeric::odeint;

void reference_model( const state_type &x , state_type &dxdt , double t )
{

	double m = 1;
	double d = 10;
	double k = 1;

	dxdt[0 ] = x[7 ];
	dxdt[1 ] = x[8 ];
	dxdt[2 ] = x[9 ];
	dxdt[3 ] = x[10];
	dxdt[4 ] = x[11];
	dxdt[5 ] = x[12];
	dxdt[6 ] = x[13];

	//             f_r		 qd_m      q_m
	dxdt[7 ] = m*( x[14] - d*x[7 ] - k*x[0 ] );
	dxdt[8 ] = m*( x[15] - d*x[8 ] - k*x[1 ] );
	dxdt[9 ] = m*( x[16] - d*x[9 ] - k*x[2 ] );
	dxdt[10] = m*( x[17] - d*x[10] - k*x[3 ] );
	dxdt[11] = m*( x[18] - d*x[11] - k*x[4 ] );
	dxdt[12] = m*( x[19] - d*x[12] - k*x[5 ] );
	dxdt[13] = m*( x[20] - d*x[13] - k*x[6 ] );

	dxdt[14] = x[14];
	dxdt[15] = x[15];
	dxdt[16] = x[16];
	dxdt[17] = x[17];
	dxdt[18] = x[18];
	dxdt[19] = x[19];
	dxdt[20] = x[20];

}

void vanderpol_model( const state_type_4 &x , state_type_4 &dxdt , double t )
{
	double Mu1 = 0.2;
	double Mu2 = 5  ;

	dxdt[0 ] = x[2];
	dxdt[1 ] = x[3];
	dxdt[2 ] = Mu1*(1-x[0]*x[0])*x[2]-x[0];
	dxdt[3 ] = Mu2*(1-x[1]*x[1])*x[3]-x[1];

}

void write_lorenz( const state_type &x , const double t )
{
    cout << t << '\t' << x[0] << '\t' << x[1] << '\t' << x[2] << endl;
}







/// Controller initialization in non-realtime
bool PR2NeuroadptControllerClass::init( pr2_mechanism_model::RobotState *robot, ros::NodeHandle &n )
{
  // Get the root and tip link names from parameter server.
  std::string root_name, tip_name;
  if (!n.getParam("root_name", root_name))
  {
    ROS_ERROR("No root name given in namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }
  if (!n.getParam("tip_name", tip_name))
  {
    ROS_ERROR("No tip name given in namespace: %s)",
              n.getNamespace().c_str());
    return false;
  }

  // Construct a chain from the root to the tip and prepare the kinematics.
  // Note the joints must be calibrated.
  if (!chain_.init(robot, root_name, tip_name))
  {
    ROS_ERROR("MyCartController could not use the chain from '%s' to '%s'",
              root_name.c_str(), tip_name.c_str());
    return false;
  }

  std::string urdf_param_ = "/robot_description";
  std::string urdf_string;

  if (!n.getParam(urdf_param_, urdf_string))
  {
    ROS_ERROR("URDF not loaded from parameter: %s)", urdf_param_.c_str());
    return false;
  }

  if (!urdf_model.initString(urdf_string))
  {
	ROS_ERROR("Failed to parse URDF file");
    return -1;
  }else {
	ROS_INFO("Successfully parsed URDF file");
  }

  std::string nn_kappa            = "nn_kappa"            ;
  std::string nn_Kv               = "nn_Kv"               ;
  std::string nn_lambda           = "nn_lambda"           ;
  std::string nn_Kz               = "nn_Kz"               ;
  std::string nn_Zb               = "nn_Zb"               ;
  std::string nn_feedForwardForce = "nn_feedForwardForce" ;
  std::string nn_nnF              = "nn_nnF"              ;
  std::string nn_nnG              = "nn_nnG"              ;

  if (!n.getParam( nn_kappa            , kappa            ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_kappa.c_str())			 ; return false; }
  if (!n.getParam( nn_Kv               , Kv               ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_Kv.c_str())				 ; return false; }
  if (!n.getParam( nn_lambda           , lambda           ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_lambda.c_str())			 ; return false; }
  if (!n.getParam( nn_Kz               , Kz               ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_Kz.c_str())				 ; return false; }
  if (!n.getParam( nn_Zb               , Zb               ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_Zb.c_str())				 ; return false; }
  if (!n.getParam( nn_feedForwardForce , feedForwardForce ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_feedForwardForce.c_str()) ; return false; }
  if (!n.getParam( nn_nnF              , nnF              ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_nnF.c_str())				 ; return false; }
  if (!n.getParam( nn_nnG              , nnG              ))
  { ROS_ERROR("Value not loaded from parameter: %s !)", nn_nnG.c_str())				 ; return false; }

  // Store the robot handle for later use (to get time).
  robot_state_ = robot;

  // Construct the kdl solvers in non-realtime.
  chain_.toKDL(kdl_chain_);
  jnt_to_pose_solver_.reset(new KDL::ChainFkSolverPos_recursive(kdl_chain_));
  jnt_to_jac_solver_.reset(new KDL::ChainJntToJacSolver(kdl_chain_));

  // Resize (pre-allocate) the variables in non-realtime.
  q_.resize(kdl_chain_.getNrOfJoints());
  q0_.resize(kdl_chain_.getNrOfJoints());
  qdot_.resize(kdl_chain_.getNrOfJoints());

  tau_t.resize(kdl_chain_.getNrOfJoints());
  tau_h.resize(kdl_chain_.getNrOfJoints());

  q_m_.resize(kdl_chain_.getNrOfJoints());
  q0_m_.resize(kdl_chain_.getNrOfJoints());
  qd_m_.resize(kdl_chain_.getNrOfJoints());
  qdd_m_.resize(kdl_chain_.getNrOfJoints());

  q_lower.resize(kdl_chain_.getNrOfJoints());
  q_upper.resize(kdl_chain_.getNrOfJoints());
  qd_limit.resize(kdl_chain_.getNrOfJoints());

  J_.resize(kdl_chain_.getNrOfJoints());
  J_m_.resize(kdl_chain_.getNrOfJoints());

  modelState.name.resize(kdl_chain_.getNrOfJoints());
  modelState.position.resize(kdl_chain_.getNrOfJoints());
  modelState.velocity.resize(kdl_chain_.getNrOfJoints());
  modelState.effort.resize(kdl_chain_.getNrOfJoints());

  robotState.name.resize(kdl_chain_.getNrOfJoints());
  robotState.position.resize(kdl_chain_.getNrOfJoints());
  robotState.velocity.resize(kdl_chain_.getNrOfJoints());
  robotState.effort.resize(kdl_chain_.getNrOfJoints());

  modelState.name[0] = kdl_chain_.getSegment(0).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[1] = kdl_chain_.getSegment(1).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[2] = kdl_chain_.getSegment(2).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[3] = kdl_chain_.getSegment(4).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[4] = kdl_chain_.getSegment(6).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[5] = kdl_chain_.getSegment(7).getJoint().getName(); // TODO test this stuff, better way to get joint names...
  modelState.name[6] = kdl_chain_.getSegment(8).getJoint().getName(); // TODO test this stuff, better way to get joint names...

  q_lower(0) = urdf_model.getJoint("r_shoulder_pan_joint"  )->limits->lower;
  q_lower(1) = urdf_model.getJoint("r_shoulder_lift_joint" )->limits->lower;
  q_lower(2) = urdf_model.getJoint("r_upper_arm_roll_joint")->limits->lower;
  q_lower(3) = urdf_model.getJoint("r_elbow_flex_joint"    )->limits->lower;
  q_lower(4) = urdf_model.getJoint("r_forearm_roll_joint"  )->limits->lower;
  q_lower(5) = urdf_model.getJoint("r_wrist_flex_joint"    )->limits->lower;
  q_lower(6) = urdf_model.getJoint("r_wrist_roll_joint"    )->limits->lower;

  q_upper(0) = urdf_model.getJoint("r_shoulder_pan_joint"  )->limits->upper;
  q_upper(1) = urdf_model.getJoint("r_shoulder_lift_joint" )->limits->upper;
  q_upper(2) = urdf_model.getJoint("r_upper_arm_roll_joint")->limits->upper;
  q_upper(3) = urdf_model.getJoint("r_elbow_flex_joint"    )->limits->upper;
  q_upper(4) = urdf_model.getJoint("r_forearm_roll_joint"  )->limits->upper;
  q_upper(5) = urdf_model.getJoint("r_wrist_flex_joint"    )->limits->upper;
  q_upper(6) = urdf_model.getJoint("r_wrist_roll_joint"    )->limits->upper;

  qd_limit(0) = urdf_model.getJoint("r_shoulder_pan_joint"  )->limits->velocity;
  qd_limit(1) = urdf_model.getJoint("r_shoulder_lift_joint" )->limits->velocity;
  qd_limit(2) = urdf_model.getJoint("r_upper_arm_roll_joint")->limits->velocity;
  qd_limit(3) = urdf_model.getJoint("r_elbow_flex_joint"    )->limits->velocity;
  qd_limit(4) = urdf_model.getJoint("r_forearm_roll_joint"  )->limits->velocity;
  qd_limit(5) = urdf_model.getJoint("r_wrist_flex_joint"    )->limits->velocity;
  qd_limit(6) = urdf_model.getJoint("r_wrist_roll_joint"    )->limits->velocity;

  // Pick the gains.
  Kp_.vel(0) = 100.0;  Kd_.vel(0) = 1.0;        // Translation x
  Kp_.vel(1) = 100.0;  Kd_.vel(1) = 1.0;        // Translation y
  Kp_.vel(2) = 100.0;  Kd_.vel(2) = 1.0;        // Translation z
  Kp_.rot(0) = 100.0;  Kd_.rot(0) = 1.0;        // Rotation x
  Kp_.rot(1) = 100.0;  Kd_.rot(1) = 1.0;        // Rotation y
  Kp_.rot(2) = 100.0;  Kd_.rot(2) = 1.0;        // Rotation z

	/////////////////////////
	// System Model

	double m = 1;
	double d = 10;
	double k = 1;

	Mm << m, 0, 0, 0, 0, 0, 0,
		  0, m, 0, 0, 0, 0, 0,
		  0, 0, m, 0, 0, 0, 0,
		  0, 0, 0, m, 0, 0, 0,
		  0, 0, 0, 0, m, 0, 0,
		  0, 0, 0, 0, 0, m, 0,
		  0, 0, 0, 0, 0, 0, m;

	Dm << d, 0, 0, 0, 0, 0, 0,
		  0, d, 0, 0, 0, 0, 0,
		  0, 0, d, 0, 0, 0, 0,
		  0, 0, 0, d, 0, 0, 0,
		  0, 0, 0, 0, d, 0, 0,
		  0, 0, 0, 0, 0, d, 0,
		  0, 0, 0, 0, 0, 0, d;

	Km << k, 0, 0, 0, 0, 0, 0,
		  0, k, 0, 0, 0, 0, 0,
		  0, 0, k, 0, 0, 0, 0,
		  0, 0, 0, k, 0, 0, 0,
		  0, 0, 0, 0, k, 0, 0,
		  0, 0, 0, 0, 0, k, 0,
		  0, 0, 0, 0, 0, 0, k;

	q_m   << 0, 0, 0, 0, 0, 0, 0 ;
	qd_m  << 0, 0, 0, 0, 0, 0, 0 ;
	qdd_m << 0, 0, 0, 0, 0, 0, 0 ;

	t_r   << 0, 0, 0, 0, 0, 0, 0 ;

	MmInv = Mm;

	delT  = 0.001;

	// initial conditions
	ode_init_x[0 ] = 0.0;
	ode_init_x[1 ] = 0.0;
	ode_init_x[2 ] = 0.0;
	ode_init_x[3 ] = 0.0;
	ode_init_x[4 ] = 0.0;
	ode_init_x[5 ] = 0.0;
	ode_init_x[6 ] = 0.0;

	ode_init_x[7 ] = 0.0;
	ode_init_x[8 ] = 0.0;
	ode_init_x[9 ] = 0.0;
	ode_init_x[10] = 0.0;
	ode_init_x[11] = 0.0;
	ode_init_x[12] = 0.0;
	ode_init_x[13] = 0.0;

	ode_init_x[14] = 0.0;
	ode_init_x[15] = 0.0;
	ode_init_x[16] = 0.0;
	ode_init_x[17] = 0.0;
	ode_init_x[18] = 0.0;
	ode_init_x[19] = 0.0;
	ode_init_x[20] = 0.0;


	vpol_init_x[0 ] = 2.0;
	vpol_init_x[1 ] = 2.0;
    vpol_init_x[2 ] = 0.0;
    vpol_init_x[3 ] = 0.0;

	// System Model END
	/////////////////////////


	/////////////////////////
	// NN

	kappa            = 0.07 ;
	Kv               = 10   ;  // prop. gain for PID inner loop
	lambda           = 0.5  ; //*std::sqrt(Kp); // der. gain for PID inner loop
	Kz               = 0    ;
	Zb               = 100  ;
	feedForwardForce = 1    ;
	nnF              = 100  ;
	nnG              = 20   ;

	hiddenLayerIdentity.setIdentity();

	W_trans.setZero();
	W_trans_next.setZero();
	V_trans.setZero();
	V_trans_next.setZero();

	F.setIdentity();
	G.setIdentity();
	L.setIdentity();

	// Very important
	Z.setZero();

	F = nnF*F;
	G = nnG*G;

	// NN END
	/////////////////////////

  /* get a handle to the hardware interface */
  pr2_hardware_interface::HardwareInterface* hardwareInterface = robot->model_->hw_;
  if(!hardwareInterface)
      ROS_ERROR("Something wrong with the hardware interface pointer!");

  l_ft_handle_ = hardwareInterface->getForceTorque("l_gripper_motor");
  r_ft_handle_ = hardwareInterface->getForceTorque("r_gripper_motor");

  if( !l_ft_handle_ /*wristFTdata.getLeftHandle()*/ )
      ROS_ERROR("Something wrong with getting l_ft handle");
  if( !r_ft_handle_ /*wristFTdata.getRightHandle()*/ )
      ROS_ERROR("Something wrong with getting r_ft handle");

  pub_cycle_count_ = 0;
  should_publish_  = false;

  // Initialize realtime publisher to publish to ROS topic
  pub_.init(n, "force_torque_stats", 1);
  pubModelStates_.init(n, "model_joint_states", 1);
  pubRobotStates_.init(n, "robot_joint_states", 1);
  pubModelCartPos_.init(n, "model_cart_pos", 1);
  pubRobotCartPos_.init(n, "robot_cart_pos", 1);
  pubControllerParam_.init(n, "controller_params", 1);

  return true;
}

/// Controller startup in realtime
void PR2NeuroadptControllerClass::starting()
{
  // Get the current joint values to compute the initial tip location.
  chain_.getPositions(q0_);
  q0_m_ = q0_;
  // Model initial conditions
  q_m = JointKdl2Eigen(q0_m_);

  jnt_to_pose_solver_->JntToCart(q0_, x0_);
  x0_m_ = x0_;

  // Initialize the phase of the circle as zero.
  circle_phase_ = 0.0;

  // Also reset the time-of-last-servo-cycle.
  last_time_ = robot_state_->getTime();

  // set FT sensor bias due to gravity
  std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
  l_ft_samples    = l_ftData_vector.size() - 1;
  l_ftBias.wrench = l_ftData_vector[l_ft_samples];

  std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
  r_ft_samples    = r_ftData_vector.size() - 1;
  r_ftBias.wrench = r_ftData_vector[r_ft_samples];
}


/// Controller update loop in realtime
void PR2NeuroadptControllerClass::update()
{

//  wristFTdata.update();
	std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
	l_ft_samples    = l_ftData_vector.size() - 1;
//      l_ftData.wrench = l_ftData_vector[l_ft_samples];
	l_ftData.wrench.force.x  = l_ftData_vector[l_ft_samples].force.x  - l_ftBias.wrench.force.x ;
	l_ftData.wrench.force.y  = l_ftData_vector[l_ft_samples].force.y  - l_ftBias.wrench.force.y ;
	l_ftData.wrench.force.z  = l_ftData_vector[l_ft_samples].force.z  - l_ftBias.wrench.force.z ;
	l_ftData.wrench.torque.x = l_ftData_vector[l_ft_samples].torque.x - l_ftBias.wrench.torque.x;
	l_ftData.wrench.torque.y = l_ftData_vector[l_ft_samples].torque.y - l_ftBias.wrench.torque.y;
	l_ftData.wrench.torque.z = l_ftData_vector[l_ft_samples].torque.z - l_ftBias.wrench.torque.z;

	std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
	r_ft_samples    = r_ftData_vector.size() - 1;
//      r_ftData.wrench = r_ftData_vector[r_ft_samples];
	r_ftData.wrench.force.x  = r_ftData_vector[r_ft_samples].force.x  - r_ftBias.wrench.force.x ;
	r_ftData.wrench.force.y  = r_ftData_vector[r_ft_samples].force.y  - r_ftBias.wrench.force.y ;
	r_ftData.wrench.force.z  = r_ftData_vector[r_ft_samples].force.z  - r_ftBias.wrench.force.z ;
	r_ftData.wrench.torque.x = r_ftData_vector[r_ft_samples].torque.x - r_ftBias.wrench.torque.x;
	r_ftData.wrench.torque.y = r_ftData_vector[r_ft_samples].torque.y - r_ftBias.wrench.torque.y;
	r_ftData.wrench.torque.z = r_ftData_vector[r_ft_samples].torque.z - r_ftBias.wrench.torque.z;


  double dt;                    // Servo loop time step

  // Calculate the dt between servo cycles.
  dt = (robot_state_->getTime() - last_time_).toSec();
  last_time_ = robot_state_->getTime();

  // Get the current joint positions and velocities.
  chain_.getPositions(q_);
  chain_.getVelocities(qdot_);

  // Save model joints to KDL
  q_m_ = JointEigen2Kdl(q_m);

  // Compute the forward kinematics and Jacobian (at this location).
  jnt_to_pose_solver_->JntToCart(q_, x_);
  jnt_to_jac_solver_->JntToJac(q_, J_);

  // Compute the forward kinematics and Jacobian of the model (at this location).
  jnt_to_pose_solver_->JntToCart(q_m_, x_m_);
  jnt_to_jac_solver_->JntToJac(q_m_, J_m_);


  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    xdot_(i) = 0;
    for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++)
      xdot_(i) += J_(i,j) * qdot_.qdot(j);
  }

  // Follow a circle of 10cm at 3 rad/sec.
  circle_phase_ += 1 * dt;
  KDL::Vector  circle(0,0,0);
  circle(2) = 0.1 * sin(circle_phase_);
  circle(1) = 0.1 * (cos(circle_phase_) - 1);

  xd_ = x0_;
  xd_.p += circle;

  // Calculate a Cartesian restoring force.
  xerr_.vel = x_.p - xd_.p;
  xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
                     xd_.M.UnitY() * x_.M.UnitY() +
                     xd_.M.UnitZ() * x_.M.UnitZ());

  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    F_(i) = - Kp_(i) * xerr_(i) - Kd_(i) * xdot_(i);
  }

  // Force control only ferr Z in ft sensor frame is x in robot frame
//  F_(0) = ferr_(2); // - Kd_(i) * xdot_(i);


  // Human force input
  // Force error
  ferr_(0) = r_ftData.wrench.force.x ; // 5*sin(circle_phase_);
  ferr_(1) = r_ftData.wrench.force.y ; // 0				      ;
  ferr_(2) = r_ftData.wrench.force.z ; // 0				      ;
  ferr_(3) = r_ftData.wrench.torque.x; // 0                   ;
  ferr_(4) = r_ftData.wrench.torque.y; // 0                   ;
  ferr_(5) = r_ftData.wrench.torque.z; // 0                   ;

  // Convert the force into a set of joint torques.
  for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++)
  {
    tau_t(i) = 0;
    tau_h(i) = 0;
    for (unsigned int j = 0 ; j < 6 ; j++)
    {
      tau_t(i) += J_(j,i) * F_(j);   // This will give the impedance to a trajectory
      tau_h(i)+= J_(j,i) * ferr_(j); // this will give the torque from human interaction
    }
  }

    /////////////////////////
	// System Model

  	// Integrator
//	t_h(0) = 0 ; // tau_h(0);
//	t_h(1) = vpol_init_x[0]; //sin(circle_phase_);    // tau_h(1);
//	t_h(2) = 0 ; // tau_h(2);
//	t_h(3) = 0 ; // tau_h(3);
//	t_h(4) = 0 ; // tau_h(4);
//	t_h(5) = 0 ; // tau_h(5);
//	t_h(6) = 0 ; // tau_h(6);

  // Reference torque from human interaction or trajectory following

  // Human
	t_r(0) = tau_h(0);
	t_r(1) = tau_h(1);
	t_r(2) = tau_h(2);
	t_r(3) = tau_h(3);
	t_r(4) = tau_h(4);
	t_r(5) = tau_h(5);
	t_r(6) = tau_h(6);

//    // Trajectory/impedance
//	t_r(0) = tau_t(0);
//	t_r(1) = tau_t(1);
//	t_r(2) = tau_t(2);
//	t_r(3) = tau_t(3);
//	t_r(4) = tau_t(4);
//	t_r(5) = tau_t(5);
//	t_r(6) = tau_t(6);


	// Current joint positions and velocities
	q = JointKdl2Eigen( q_ );
	qd = JointVelKdl2Eigen( qdot_ );

	q_m   = q_m + delT*qd_m;
	qd_m  = qd_m + delT*qdd_m;
	qdd_m = MmInv*( t_r - Dm*qd_m - Km*q_m );

	// Check for joint limits and reset
	// (condition) ? (if_true) : (if_false)
	q_m(0) = fmax( (double) q_m(0), (double) q_lower(0) );
	q_m(1) = fmax( (double) q_m(1), (double) q_lower(1) );
	q_m(2) = fmax( (double) q_m(2), (double) q_lower(2) );
	q_m(3) = fmax( (double) q_m(3), (double) q_lower(3) );
//	q_m(4) = fmax( (double) q_m(4), (double) q_lower(4) );
	q_m(5) = fmax( (double) q_m(5), (double) q_lower(5) );
//	q_m(6) = fmax( (double) q_m(6), (double) q_lower(6) );

	q_m(0) = fmin( (double) q_m(0), (double) q_upper(0) );
	q_m(1) = fmin( (double) q_m(1), (double) q_upper(1) );
	q_m(2) = fmin( (double) q_m(2), (double) q_upper(2) );
	q_m(3) = fmin( (double) q_m(3), (double) q_upper(3) );
//	q_m(4) = fmin( (double) q_m(4), (double) q_upper(4) );
	q_m(5) = fmin( (double) q_m(5), (double) q_upper(5) );
//	q_m(6) = fmin( (double) q_m(6), (double) q_upper(6) );

	qd_m(0) = fmin( (double) qd_m(0), (double) qd_limit(0) );
	qd_m(1) = fmin( (double) qd_m(1), (double) qd_limit(1) );
	qd_m(2) = fmin( (double) qd_m(2), (double) qd_limit(2) );
	qd_m(3) = fmin( (double) qd_m(3), (double) qd_limit(3) );
	qd_m(4) = fmin( (double) qd_m(4), (double) qd_limit(4) );
	qd_m(5) = fmin( (double) qd_m(5), (double) qd_limit(5) );
	qd_m(6) = fmin( (double) qd_m(6), (double) qd_limit(6) );

//	ode_init_x[0 ] = 0.0;
//	ode_init_x[1 ] = 0.0;
//	ode_init_x[2 ] = 0.0;
//	ode_init_x[3 ] = 0.0;
//	ode_init_x[4 ] = 0.0;
//	ode_init_x[5 ] = 0.0;
//	ode_init_x[6 ] = 0.0;
//
//	ode_init_x[7 ] = 0.0;
//	ode_init_x[8 ] = 0.0;
//	ode_init_x[9 ] = 0.0;
//	ode_init_x[10] = 0.0;
//	ode_init_x[11] = 0.0;
//	ode_init_x[12] = 0.0;
//	ode_init_x[13] = 0.0;

	ode_init_x[14] = t_r(0);
	ode_init_x[15] = t_r(1);
	ode_init_x[16] = t_r(2);
	ode_init_x[17] = t_r(3);
	ode_init_x[18] = t_r(4);
	ode_init_x[19] = t_r(5);
	ode_init_x[20] = t_r(6);

//	integrate( reference_model , ode_init_x , 0.0 , 0.001 , 0.001 );
	integrate( vanderpol_model , vpol_init_x , 0.0 , 0.001 , 0.001 );

	// System Model END
	/////////////////////////

//	// DEBUG
//	q_m(0)  = 0 ; //- 0.5 * (sin(circle_phase_) + 1 );
//	q_m(1)  = 0 ; //- 0.5 * (sin(circle_phase_) + 1 );
//	q_m(2)  = 0 ; //- 0.5 * (sin(circle_phase_) + 1 );
//	q_m(3)  =       - 0.5 * (sin(circle_phase_) + 1.5 );
//	q_m(4)  = 0;
//	q_m(5)  = 0;
//	q_m(6)  = 0;
//
//	qd_m(0) = 0;
//	qd_m(1) = 0;
//	qd_m(2) = 0;
//	qd_m(3) = - 10 * (cos(circle_phase_));
//	qd_m(4) = 0;
//	qd_m(5) = 0;
//	qd_m(6) = 0;


    /////////////////////////
	// NN

	W_trans = W_trans_next;
	V_trans = V_trans_next;

	// Filtered error
	r = lambda*(qd_m - qd) + (q_m - q);

	// Robust term
	Z.block(0,0,Hidden,Outputs) = W_trans.transpose();
	Z.block(Hidden,Outputs,Inputs+1,Hidden) = V_trans.transpose();
	vRobust = - Kz*(Z.norm() + Zb)*r;

	x(0 ) =  q(0);
	x(1 ) =  q(1);
	x(2 ) =  q(2);
	x(3 ) =  q(3);
	x(4 ) =  q(4);
	x(5 ) =  q(5);
	x(6 ) =  q(6);
	x(7 ) = qd(0);
	x(8 ) = qd(1);
	x(9 ) = qd(2);
	x(10) = qd(3);
	x(11) = qd(4);
	x(12) = qd(5);
	x(13) = qd(6);
	x(14) = 1;

	hiddenLayer_in = V_trans*x;
	hiddenLayer_out = sigmoid(hiddenLayer_in);
	outputLayer_out = W_trans*hiddenLayer_out;

	y = outputLayer_out;

	// control torques
	tau = Kv*r + y - vRobust - feedForwardForce*t_r;
//	tau = (qd_m - qd) + 100*(q_m - q);

	//
	sigmaPrime = hiddenLayer_out.asDiagonal()*( hiddenLayerIdentity - hiddenLayerIdentity*hiddenLayer_out.asDiagonal() );

	// Wk+1                  = Wk                  +  Wkdot                                                                                                          * dt
	W_trans_next.transpose() = W_trans.transpose() + (F*hiddenLayer_out*r.transpose() - F*sigmaPrime*V_trans*x*r.transpose() - kappa*F*r.norm()*W_trans.transpose()) * delT;

	// Vk+1                  = Vk                  +  Vkdot                                                                                      			 * dt
	V_trans_next.transpose() = V_trans.transpose() + (G*x*(sigmaPrime.transpose()*W_trans.transpose()*r).transpose() - kappa*G*r.norm()*V_trans.transpose()) * delT;

	// Convert from Eigen to KDL
	tau_t = JointEigen2Kdl( tau );

	// NN END
	/////////////////////////


	modelState.header.stamp = robot_state_->getTime();
	robotState.header.stamp = robot_state_->getTime();

	modelState.position[0] = q_m(0);
	modelState.position[1] = q_m(1);
	modelState.position[2] = q_m(2);
	modelState.position[3] = q_m(3);
	modelState.position[4] = q_m(4);
	modelState.position[5] = q_m(5);
	modelState.position[6] = q_m(6);

	modelState.velocity[0] = qd_m(0); // ode_init_x[0 ] ; // tau(0);
	modelState.velocity[1] = qd_m(1); // ode_init_x[1 ] ; // tau(1);
	modelState.velocity[2] = qd_m(2); // ode_init_x[2 ] ; // tau(2);
	modelState.velocity[3] = qd_m(3); // ode_init_x[3 ] ; // tau(3);
	modelState.velocity[4] = qd_m(4); // ode_init_x[4 ] ; // tau(4);
	modelState.velocity[5] = qd_m(5); // ode_init_x[5 ] ; // tau(5);
	modelState.velocity[6] = qd_m(6); // ode_init_x[6 ] ; // tau(6);

	// Input torque to mode | torque from human
	modelState.effort[0] = t_r(0);
	modelState.effort[1] = t_r(1);
	modelState.effort[2] = t_r(2);
	modelState.effort[3] = t_r(3);
	modelState.effort[4] = t_r(4);
	modelState.effort[5] = t_r(5);
	modelState.effort[6] = t_r(6);

	robotState.position[0] = q(0);;
	robotState.position[1] = q(1);;
	robotState.position[2] = q(2);;
	robotState.position[3] = q(3);;
	robotState.position[4] = q(4);;
	robotState.position[5] = q(5);;
	robotState.position[6] = q(6);;

	robotState.velocity[0] = qd(0);
	robotState.velocity[1] = qd(1);
	robotState.velocity[2] = qd(2);
	robotState.velocity[3] = qd(3);
	robotState.velocity[4] = qd(4);
	robotState.velocity[5] = qd(5);
	robotState.velocity[6] = qd(6);

	// Output torque from controller that is sent to the robot
	robotState.effort[0] = tau_t(0);
	robotState.effort[1] = tau_t(1);
	robotState.effort[2] = tau_t(2);
	robotState.effort[3] = tau_t(3);
	robotState.effort[4] = tau_t(4);
	robotState.effort[5] = tau_t(5);
	robotState.effort[6] = tau_t(6);


	// And finally send these torques out.
    chain_.setEfforts(tau_t);

    // Publish data in ROS message every 10 cycles (about 100Hz)
	if (++pub_cycle_count_ > 10)
	{
		should_publish_ = true;
		pub_cycle_count_ = 0;
	}

	if (should_publish_ && pub_.trylock())
	{
		should_publish_ = false;

		pub_.msg_.header.stamp = robot_state_->getTime();
		pub_.msg_.wrench = r_ftData.wrench; // wristFTdata.getRightData().wrench;

		pubModelStates_.msg_ = modelState;
		pubRobotStates_.msg_ = robotState;

		pubControllerParam_.msg_.kappa            = kappa            ;
		pubControllerParam_.msg_.Kv               = Kv               ;
		pubControllerParam_.msg_.lambda           = lambda           ;
		pubControllerParam_.msg_.Kz               = Kz               ;
		pubControllerParam_.msg_.Zb               = Zb               ;
		pubControllerParam_.msg_.feedForwardForce = feedForwardForce ;
		pubControllerParam_.msg_.F				  = nnF              ;
		pubControllerParam_.msg_.G				  = nnG              ;
		pubControllerParam_.msg_.inParams   	  = Inputs           ;
		pubControllerParam_.msg_.outParams		  = Outputs          ;
		pubControllerParam_.msg_.hiddenNodes	  = Hidden           ;
		pubControllerParam_.msg_.errorParams      = Error  			 ;

		pubControllerParam_.msg_.m				  = Mm(0,0)          ;
		pubControllerParam_.msg_.d				  = Dm(0,0)          ;
		pubControllerParam_.msg_.k				  = Km(0,0)          ;

		pubModelCartPos_.msg_.header.stamp = robot_state_->getTime();
		pubRobotCartPos_.msg_.header.stamp = robot_state_->getTime();

		tf::PoseKDLToMsg(x_m_, modelCartPos_);
		tf::PoseKDLToMsg(x_  , robotCartPos_);

		pubModelCartPos_.msg_.pose = modelCartPos_;
		pubRobotCartPos_.msg_.pose = robotCartPos_;

		pub_.unlockAndPublish();
		pubModelStates_.unlockAndPublish();
		pubRobotStates_.unlockAndPublish();
		pubModelCartPos_.unlockAndPublish();
		pubModelCartPos_.unlockAndPublish();
		pubControllerParam_.unlockAndPublish();
	}

}

PR2NeuroadptControllerClass::SystemVector
PR2NeuroadptControllerClass::JointKdl2Eigen( KDL::JntArray & joint_ )
{
	SystemVector joint;
	joint(0) = joint_(0);
	joint(1) = joint_(1);
	joint(2) = joint_(2);
	joint(3) = joint_(3);
	joint(4) = joint_(4);
	joint(5) = joint_(5);
	joint(6) = joint_(6);

	return joint;
}

PR2NeuroadptControllerClass::SystemVector
PR2NeuroadptControllerClass::JointVelKdl2Eigen( KDL::JntArrayVel & joint_ )
{
	SystemVector joint;
	joint(0) = joint_.qdot(0);
	joint(1) = joint_.qdot(1);
	joint(2) = joint_.qdot(2);
	joint(3) = joint_.qdot(3);
	joint(4) = joint_.qdot(4);
	joint(5) = joint_.qdot(5);
	joint(6) = joint_.qdot(6);

	return joint;
}

KDL::JntArray
PR2NeuroadptControllerClass::JointEigen2Kdl( SystemVector & joint )
{
	KDL::JntArray joint_;
	joint_.resize(7);

	joint (0) = joint(0);
	joint_(1) = joint(1);
	joint_(2) = joint(2);
	joint_(3) = joint(3);
	joint_(4) = joint(4);
	joint_(5) = joint(5);
	joint_(6) = joint(6);

	return joint_;
}

Eigen::Matrix<double, PR2NeuroadptControllerClass::Hidden, 1>
PR2NeuroadptControllerClass::sigmoid( Eigen::Matrix<double, Hidden, 1> & z )
{
  for(uint i=0;i<z.size();i++)
  {
	z(i) = 1.0/(1.0 + exp(-(double)z(i)));
  }
  return z;
}



/// Controller stopping in realtime
void PR2NeuroadptControllerClass::stopping()
{}


// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS( PR2NeuroadptControllerClass,
		          	      pr2_controller_ns::PR2NeuroadptControllerClass,
                          pr2_controller_interface::Controller )
