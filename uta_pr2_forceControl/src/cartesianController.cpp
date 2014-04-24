#include "uta_pr2_forceControl/cartesianController.h"
#include <pluginlib/class_list_macros.h>
#include "oel/least_squares.hpp"
#include "pinv.hpp"

using namespace pr2_controller_ns;

/// Controller initialization in non-realtime
bool PR2CartesianControllerClass::init(pr2_mechanism_model::RobotState *robot,
                                 ros::NodeHandle &n)
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

  double cartPos_Kp_x = 0;
  double cartPos_Kp_y = 0;
  double cartPos_Kp_z = 0;
  double cartPos_Kd_x = 0;
  double cartPos_Kd_y = 0;
  double cartPos_Kd_z = 0;

  double cartRot_Kp_x = 0;
  double cartRot_Kp_y = 0;
  double cartRot_Kp_z = 0;
  double cartRot_Kd_x = 0;
  double cartRot_Kd_y = 0;
  double cartRot_Kd_z = 0;

  if (!n.getParam( para_cartPos_Kp_x , cartPos_Kp_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kp_x.c_str()) ; return false; }
  if (!n.getParam( para_cartPos_Kp_y , cartPos_Kp_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kp_y.c_str()) ; return false; }
  if (!n.getParam( para_cartPos_Kp_z , cartPos_Kp_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kp_z.c_str()) ; return false; }
  if (!n.getParam( para_cartPos_Kd_x , cartPos_Kd_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kd_x.c_str()) ; return false; }
  if (!n.getParam( para_cartPos_Kd_y , cartPos_Kd_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kd_y.c_str()) ; return false; }
  if (!n.getParam( para_cartPos_Kd_z , cartPos_Kd_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartPos_Kd_z.c_str()) ; return false; }

  if (!n.getParam( para_cartRot_Kp_x , cartRot_Kp_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kp_x.c_str()) ; return false; }
  if (!n.getParam( para_cartRot_Kp_y , cartRot_Kp_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kp_y.c_str()) ; return false; }
  if (!n.getParam( para_cartRot_Kp_z , cartRot_Kp_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kp_z.c_str()) ; return false; }
  if (!n.getParam( para_cartRot_Kd_x , cartRot_Kd_x )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kd_x.c_str()) ; return false; }
  if (!n.getParam( para_cartRot_Kd_y , cartRot_Kd_y )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kd_y.c_str()) ; return false; }
  if (!n.getParam( para_cartRot_Kd_z , cartRot_Kd_z )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartRot_Kd_z.c_str()) ; return false; }


  // Desired cartesian pose
  cartDesX     = 0.0           ;
  cartDesY     = 0.1           ;
  cartDesZ     = 0.1           ;
  cartDesRoll  = 0             ;
  cartDesPitch = 1.57079632679 ;
  cartDesYaw   = 0             ;

  std::string para_cartDesX     = "/cartDesX";
  std::string para_cartDesY     = "/cartDesY";
  std::string para_cartDesZ     = "/cartDesZ";
  std::string para_cartDesRoll  = "/cartDesRoll";
  std::string para_cartDesPitch = "/cartDesPitch";
  std::string para_cartDesYaw   = "/cartDesYaw";

  if (!n.getParam( para_cartDesX     , cartDesX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesX    .c_str()) ; return false; }
  if (!n.getParam( para_cartDesY     , cartDesY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesY    .c_str()) ; return false; }
  if (!n.getParam( para_cartDesZ     , cartDesZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesZ    .c_str()) ; return false; }
  if (!n.getParam( para_cartDesRoll  , cartDesRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesRoll .c_str()) ; return false; }
  if (!n.getParam( para_cartDesPitch , cartDesPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesPitch.c_str()) ; return false; }
  if (!n.getParam( para_cartDesYaw   , cartDesYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesYaw  .c_str()) ; return false; }

  // Initial cartesian pose
  cartIniX     = 0.1           ;
  cartIniY     = 0.1           ;
  cartIniZ     = 0             ;

  std::string para_cartIniX     = "/cartIniX";
  std::string para_cartIniY     = "/cartIniY";
  std::string para_cartIniZ     = "/cartIniZ";

  if (!n.getParam( para_cartIniX, cartIniX )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniX.c_str()) ; return false; }
  if (!n.getParam( para_cartIniY, cartIniY )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniY.c_str()) ; return false; }
  if (!n.getParam( para_cartIniZ, cartIniZ )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniZ.c_str()) ; return false; }

  useCurrentCartPose = false ;
  std::string para_useCurrentCartPose     = "/useCurrentCartPose";
  if (!n.getParam( para_useCurrentCartPose, useCurrentCartPose )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_useCurrentCartPose.c_str()) ; return false; }

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
  tau_c_.resize(kdl_chain_.getNrOfJoints());
  J_.resize(kdl_chain_.getNrOfJoints());

  qnom.resize(kdl_chain_.getNrOfJoints());
  q_lower.resize(kdl_chain_.getNrOfJoints());
  q_upper.resize(kdl_chain_.getNrOfJoints());
  qd_limit.resize(kdl_chain_.getNrOfJoints());

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

  qnom(0) = ( q_upper(0) - q_lower(0) ) / 2 ;
  qnom(1) = ( q_upper(1) - q_lower(1) ) / 2 ;
  qnom(2) = ( q_upper(2) - q_lower(2) ) / 2 ;
  qnom(3) = ( q_upper(3) - q_lower(3) ) / 2 ;
  qnom(4) = ( q_upper(4) - q_lower(4) ) / 2 ;
  qnom(5) = ( q_upper(5) - q_lower(5) ) / 2 ;
  qnom(6) = ( q_upper(6) - q_lower(6) ) / 2 ;

  // Pick the gains.
//  Kp_.vel(0) = 100.0;  Kd_.vel(0) = 1.0;        // Translation x
//  Kp_.vel(1) = 000.0;  Kd_.vel(1) = 1.0;        // Translation y
//  Kp_.vel(2) = 100.0;  Kd_.vel(2) = 1.0;        // Translation z
//  Kp_.rot(0) = 100.0;  Kd_.rot(0) = 1.0;        // Rotation    x
//  Kp_.rot(1) = 100.0;  Kd_.rot(1) = 1.0;        // Rotation    y
//  Kp_.rot(2) = 100.0;  Kd_.rot(2) = 1.0;        // Rotation    z

  Jacobian         = Eigen::MatrixXd::Zero( 6, kdl_chain_.getNrOfJoints() ) ;
  JacobianPinv     = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), 6 ) ;
  JacobianTrans    = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), 6 ) ;
  JacobianTransPinv= Eigen::MatrixXd::Zero( 6, kdl_chain_.getNrOfJoints() ) ;
  nullSpace        = Eigen::MatrixXd::Zero( kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints() ) ;

  cartControlForce = Eigen::VectorXd::Zero( 6 ) ;
  nullspaceTorque  = Eigen::VectorXd::Zero( kdl_chain_.getNrOfJoints() ) ;
  controlTorque    = Eigen::VectorXd::Zero( kdl_chain_.getNrOfJoints() ) ;

  Kp_.resize( kdl_chain_.getNrOfJoints() ) ;
  Kd_.resize( kdl_chain_.getNrOfJoints() ) ;

  Kp_(0) = cartPos_Kp_x;  Kd_(0) = cartPos_Kd_x; // Translation x
  Kp_(1) = cartPos_Kp_y;  Kd_(1) = cartPos_Kd_y; // Translation y
  Kp_(2) = cartPos_Kp_z;  Kd_(2) = cartPos_Kd_z; // Translation z
  Kp_(3) = cartRot_Kp_x;  Kd_(3) = cartRot_Kd_x; // Rotation    x
  Kp_(4) = cartRot_Kp_y;  Kd_(4) = cartRot_Kd_y; // Rotation    y
  Kp_(5) = cartRot_Kp_z;  Kd_(5) = cartRot_Kd_z; // Rotation    z

//  ROS_ERROR("Joint no: %d", kdl_chain_.getNrOfJoints());

  /////////////////////////
  // DATA COLLECTION

  capture_srv_   = n.advertiseService("capture", &PR2CartesianControllerClass::capture, this);

  pubFTData_             = n.advertise< geometry_msgs::WrenchStamped             >( "FT_data"              , StoreLen);
  pubModelStates_        = n.advertise< sensor_msgs::JointState                  >( "model_joint_states"   , StoreLen);
  pubRobotStates_        = n.advertise< sensor_msgs::JointState                  >( "robot_joint_states"   , StoreLen);
  pubModelCartPos_       = n.advertise< geometry_msgs::PoseStamped               >( "model_cart_pos"       , StoreLen);
  pubRobotCartPos_       = n.advertise< geometry_msgs::PoseStamped               >( "robot_cart_pos"       , StoreLen);
  pubControllerParam_    = n.advertise< neuroadaptive_msgs::controllerParam    >( "controller_params"    , StoreLen);
  pubControllerFullData_ = n.advertise< neuroadaptive_msgs::controllerFullData >( "controllerFullData"   , StoreLen);

  storage_index_ = StoreLen;

  // DATA COLLECTION END
  /////////////////////////

  return true;
}

/// Controller startup in realtime
void PR2CartesianControllerClass::starting()
{
  // Get the current joint values to compute the initial tip location.
  chain_.getPositions(q0_);
  jnt_to_pose_solver_->JntToCart(q0_, x0_);

  /////////////////////////
  // System Model

  // System Model END
  /////////////////////////

  // Initialize the phase of the circle as zero.
  circle_phase_ = 0.0;
  startCircleTraj = true;

  // Also reset the time-of-last-servo-cycle.
  last_time_ = robot_state_->getTime();
//  start_time_ = robot_state_->getTime();
}


/// Controller update loop in realtime
void PR2CartesianControllerClass::update()
{
  double dt;                    // Servo loop time step

  // Calculate the dt between servo cycles.
  dt = (robot_state_->getTime() - last_time_).toSec();
  last_time_ = robot_state_->getTime();

  // Get the current joint positions and velocities.
  chain_.getPositions(q_);
  chain_.getVelocities(qdot_);

  // Compute the forward kinematics and Jacobian (at this location).
  jnt_to_pose_solver_->JntToCart(q_, x_);
  jnt_to_jac_solver_->JntToJac(q_, J_);

  for (unsigned int i = 0 ; i < 6 ; i++)
  {
    xdot_(i) = 0;
    for (unsigned int j = 0 ; j < kdl_chain_.getNrOfJoints() ; j++)
    {
      xdot_(i) += J_(i,j) * qdot_.qdot(j);
      // Eigen Jacobian
      Jacobian(i,j) = J_(i,j);
    }
  }

  // Follow a circle of 10cm at 3 rad/sec.
  circle_phase_ += 3.0 * dt;
  KDL::Vector  circle(cartDesX,cartDesY,cartDesZ);

  circle(1) = cartDesY * (cos(circle_phase_) - 1);
  circle(2) = cartDesZ * sin(circle_phase_)      ;

  if( useCurrentCartPose )
  {
    // Start from current
    xd_ = x0_;
  }else
  {
    // Start from specified
    xd_.p = KDL::Vector(cartIniX,cartIniY,cartIniZ);
  }

  xd_.p += circle;
  xd_.M =  KDL::Rotation::RPY( cartDesRoll, cartDesPitch, cartDesYaw );

  // Calculate a Cartesian restoring force.
  xerr_.vel = x_.p - xd_.p;
  xerr_.rot = 0.5 * (xd_.M.UnitX() * x_.M.UnitX() +
                     xd_.M.UnitY() * x_.M.UnitY() +
                     xd_.M.UnitZ() * x_.M.UnitZ());

  robotCartPos_.position.x    = x_.p(0);
  robotCartPos_.position.y    = x_.p(1);
  robotCartPos_.position.z    = x_.p(2);
  x_.M.GetQuaternion( robotCartPos_.orientation.x ,
                      robotCartPos_.orientation.y ,
                      robotCartPos_.orientation.z ,
                      robotCartPos_.orientation.w  );

  modelCartPos_.position.x    = xd_.p(0);
  modelCartPos_.position.y    = xd_.p(1);
  modelCartPos_.position.z    = xd_.p(2);
  xd_.M.GetQuaternion( modelCartPos_.orientation.x ,
                       modelCartPos_.orientation.y ,
                       modelCartPos_.orientation.z ,
                       modelCartPos_.orientation.w  );

  for (unsigned int i = 0 ; i < 6 ; i++)
    cartControlForce(i) = - Kp_(i) * xerr_(i) - Kd_(i) * xdot_(i);

//  // Convert the force into a set of joint torques.
//  for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++)
//  {
//    tau_c_(i) = 0;
//    for (unsigned int j = 0 ; j < 6 ; j++)
//      tau_c_(i) += J_(j,i) * cartControlForce(j);
//  }

  JacobianTrans = Jacobian.transpose();

  JacobianTransPinv = pseudoInverse( JacobianTrans, 0.01 ); // Jacobian * (JacobianTrans*Jacobian).inverse();
  JacobianPinv      = pseudoInverse( Jacobian, 0.01 );


  // ======== J psuedo-inverse and Nullspace computation

  double k_posture = 25.0;
  double jacobian_inverse_damping = 0.01;

  JointVec joint_dd_ff_;

  joint_dd_ff_(0) = 3.33   ;
  joint_dd_ff_(1) = 1.16   ;
  joint_dd_ff_(2) = 0.1    ;
  joint_dd_ff_(3) = 0.25   ;
  joint_dd_ff_(4) = 0.133  ;
  joint_dd_ff_(5) = 0.0727 ;
  joint_dd_ff_(6) = 0.0727 ;

  // Computes pseudo-inverse of J
  Eigen::Matrix<double,6,6> I6; I6.setIdentity();
  Eigen::Matrix<double,6,6> JJt_damped = Jacobian * JacobianTrans + jacobian_inverse_damping * I6;
  Eigen::Matrix<double,6,6> JJt_inv_damped = JJt_damped.inverse();
  Eigen::Matrix<double,7,6> J_pinv = JacobianTrans * JJt_inv_damped;

  // Computes the nullspace of J
  Eigen::Matrix<double,7,7> I; I.setIdentity();
  nullSpace = I - J_pinv * Jacobian;

  // ======== Posture control

  // Computes the desired joint torques for achieving the posture
  bool use_posture_ = true;

  nullspaceTorque.setZero();
  if (use_posture_)
  {
    JointVec posture_err ;

    posture_err(0) = qnom(0) - q_(0) ;
    posture_err(1) = qnom(1) - q_(1) ;
    posture_err(2) = qnom(2) - q_(2) ;
    posture_err(3) = qnom(3) - q_(3) ;
    posture_err(4) = qnom(4) - q_(4) ;
    posture_err(5) = qnom(5) - q_(5) ;
    posture_err(6) = qnom(6) - q_(6) ;


    for (size_t j = 0; j < 7; ++j)
    {
      if (chain_.getJoint(j)->joint_->type == urdf::Joint::CONTINUOUS)
        posture_err[j] = angles::normalize_angle(posture_err[j]);
    }

    for (size_t j = 0; j < 7; ++j) {
      if (fabs(qnom(j) - 9999) < 1e-5)
        posture_err[j] = 0.0;
    }

    JointVec qdd_posture = k_posture * posture_err;
    nullspaceTorque = joint_dd_ff_.array() * (nullSpace * qdd_posture).array();
  }


  // dynamically consistent generalized inverse is defined to
  // J^T# = (J M^−1 J^T)^-1 JM^−1


//  for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++)
//    nullspaceTorque(i) = 0.01*( qnom(i) - q_(i)) + 0.5*qdot_.qdot(i);

  controlTorque = JacobianTrans*cartControlForce + nullspaceTorque;
      // nullspace controller
      //+ ( Eigen::MatrixXd::Identity(kdl_chain_.getNrOfJoints(), kdl_chain_.getNrOfJoints()) - JacobianTrans*JacobianTransPinv )*nullspaceTorque;

  for (unsigned int i = 0 ; i < kdl_chain_.getNrOfJoints() ; i++)
    tau_c_(i) = controlTorque(i);

  // And finally send these torques out.
  chain_.setEfforts(tau_c_);

  /////////////////////////
  // DATA COLLECTION

  bufferData( dt );

  // DATA COLLECTION END
  /////////////////////////

}

void PR2CartesianControllerClass::bufferData( double & dt )
{
        int index = storage_index_;
        if ((index >= 0) && (index < StoreLen))
        {
//                tf::PoseKDLToMsg(x_m_, modelCartPos_);
//                tf::PoseKDLToMsg(x_  , robotCartPos_);

                msgControllerFullData[index].dt                = dt                          ;

                // Force Data
                msgControllerFullData[index].force_x           = 0 ;// r_ftData.wrench.force.x     ;
                msgControllerFullData[index].force_y           = 0 ;// r_ftData.wrench.force.y     ;
                msgControllerFullData[index].force_z           = 0 ;// r_ftData.wrench.force.z     ;
                msgControllerFullData[index].torque_x          = 0 ;// r_ftData.wrench.torque.x    ;
                msgControllerFullData[index].torque_y          = 0 ;// r_ftData.wrench.torque.y    ;
                msgControllerFullData[index].torque_z          = 0 ;// r_ftData.wrench.torque.z    ;

                // Input reference efforts(torques)
                msgControllerFullData[index].reference_eff_j0  = 0; //t_r(0)                      ;
                msgControllerFullData[index].reference_eff_j1  = 0; //t_r(1)                      ;
                msgControllerFullData[index].reference_eff_j2  = 0; //t_r(2)                      ;
                msgControllerFullData[index].reference_eff_j3  = 0; //t_r(3)                      ;
                msgControllerFullData[index].reference_eff_j4  = 0; //t_r(4)                      ;
                msgControllerFullData[index].reference_eff_j5  = 0; //t_r(5)                      ;
                msgControllerFullData[index].reference_eff_j6  = 0; //t_r(6)                      ;

                // Model States
                msgControllerFullData[index].m_cartPos_x       = modelCartPos_.position.x    ;
                msgControllerFullData[index].m_cartPos_y       = modelCartPos_.position.y    ;
                msgControllerFullData[index].m_cartPos_z       = modelCartPos_.position.z    ;
                msgControllerFullData[index].m_cartPos_Qx      = modelCartPos_.orientation.x ;
                msgControllerFullData[index].m_cartPos_Qy      = modelCartPos_.orientation.y ;
                msgControllerFullData[index].m_cartPos_Qz      = modelCartPos_.orientation.z ;
                msgControllerFullData[index].m_cartPos_QW      = modelCartPos_.orientation.w ;
/*
                msgControllerFullData[index].m_pos_x           = x_m(0)                      ;
                msgControllerFullData[index].m_pos_y           = x_m(1)                      ;
                msgControllerFullData[index].m_pos_z           = x_m(2)                      ;

                msgControllerFullData[index].m_vel_x           = xd_m(0)                     ;
                msgControllerFullData[index].m_vel_y           = xd_m(1)                     ;
                msgControllerFullData[index].m_vel_z           = xd_m(2)                     ;

                msgControllerFullData[index].m_acc_x           = xdd_m(0)                    ;
                msgControllerFullData[index].m_acc_y           = xdd_m(1)                    ;
                msgControllerFullData[index].m_acc_z           = xdd_m(2)                    ;

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
*/
                // Robot States
                msgControllerFullData[index].r_cartPos_x       = robotCartPos_.position.x    ;
                msgControllerFullData[index].r_cartPos_y       = robotCartPos_.position.y    ;
                msgControllerFullData[index].r_cartPos_z       = robotCartPos_.position.z    ;
                msgControllerFullData[index].r_cartPos_Qx      = robotCartPos_.orientation.x ;
                msgControllerFullData[index].r_cartPos_Qy      = robotCartPos_.orientation.y ;
                msgControllerFullData[index].r_cartPos_Qz      = robotCartPos_.orientation.z ;
                msgControllerFullData[index].r_cartPos_QW      = robotCartPos_.orientation.w ;
/*
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
*/
                // TODO fix this
                // Model Params
                msgControllerFullData[index].m                 =   0 ; // outerLoopMSDmodel.getMass(  )(0,0) ;
                msgControllerFullData[index].d                 =   0 ; // outerLoopMSDmodel.getSpring()(0,0) ;
                msgControllerFullData[index].k                 =   0 ; // outerLoopMSDmodel.getDamper()(0,0) ;

                // Increment for the next cycle.
                storage_index_ = index+1;
        }
}

/// Service call to capture and extract the data
bool PR2CartesianControllerClass::capture( std_srvs::Empty::Request& req,
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

/// Controller stopping in realtime
void PR2CartesianControllerClass::stopping()
{}

// Register controller to pluginlib
PLUGINLIB_REGISTER_CLASS( PR2CartesianControllerClass,
						  pr2_controller_ns::PR2CartesianControllerClass,
                          pr2_controller_interface::Controller )
