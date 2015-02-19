/*
 * interaction_force_estimator.cpp
 *
 *  Created on: Jan 29, 2014
 *      Author: isura
 */

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "pr2_msgs/AccelerometerState.h"
#include "geometry_msgs/WrenchStamped.h"

#include "oel/kalman_estimate.hpp"

#include <Eigen/Geometry>

class PR2ForceEstimator
{
  Eigen::Vector3d forces;
  Eigen::Vector3d torques;

  Eigen::Vector3d linear_acc;
  Eigen::Vector3d angular_vel;

  Eigen::MatrixXd A ;
  Eigen::MatrixXd B ;
  Eigen::MatrixXd H ;
  Eigen::MatrixXd G ;
  Eigen::MatrixXd Q ;
  Eigen::MatrixXd R ;
  Eigen::MatrixXd X0;
  Eigen::MatrixXd P0;

  oel::kalman::DiscreteTimeKalmanFilter kalmanForces ;
  oel::kalman::DiscreteTimeKalmanFilter kalmanTorques;

  // ROS Nodehandle
  ros::NodeHandle* n;

  ros::Publisher force_pub ;
  geometry_msgs::WrenchStamped msg;

public:
  PR2ForceEstimator()
  {
    forces      = Eigen::Vector3d::Zero();
    torques     = Eigen::Vector3d::Zero();

    linear_acc  = Eigen::Vector3d::Zero();
    angular_vel = Eigen::Vector3d::Zero();

    A.resize(2,2);
    A << 1, 1,
         0, 1;

    B.resize(2,1);
    B << 0,
         0;

    H.resize(1,2);
    H << 1, 0;

    G.resize(2,1);
    G << 0,
         1;

    Q.resize(1,1);
    Q << 1;

    R.resize(1,1);
    R << 2;

    X0.resize(2,1);
    X0 << 0,
          10;

    P0.resize(2,2);
    P0 << 2, 0,
          0, 3;

    kalmanForces .init( A, B, H, G, Q, R, X0, P0 );
    kalmanTorques.init( A, B, H, G, Q, R, X0, P0 );

    // Start up ROS
    std::string name = "interaction_force_estimator";
    int argc = 0;

    ros::init(argc, NULL, name);
    this->n = new ros::NodeHandle("~");
    force_pub = n->advertise<geometry_msgs::WrenchStamped>("/ft/r_gripper_motor_est", 1);

  }

  ~PR2ForceEstimator()
  {

  }

  void updateForces( Eigen::Vector3d & para_forces ,
                     Eigen::Vector3d & para_torques  )
  {
    forces  = para_forces  ;
    torques = para_torques ;
  }

  void updateAcc( Eigen::Vector3d & para_linear_acc ,
                  Eigen::Vector3d & para_angular_vel  )
  {
    linear_acc  = para_linear_acc  ;
    angular_vel = para_angular_vel ;
  }

  void updateForces( const geometry_msgs::WrenchStamped::ConstPtr& msg )
  {
    forces (0) = msg->wrench.force.x ;
    forces (1) = msg->wrench.force.y ;
    forces (2) = msg->wrench.force.z ;

    torques(0) = msg->wrench.torque.x ;
    torques(1) = msg->wrench.torque.y ;
    torques(2) = msg->wrench.torque.z ;
  }


  void updateAcc( const pr2_msgs::AccelerometerState::ConstPtr& msg )
  {
    linear_acc (0) = ( msg->samples[0].x + msg->samples[1].x + msg->samples[2].x )/3 ;
    linear_acc (1) = ( msg->samples[0].y + msg->samples[1].y + msg->samples[2].y )/3 ;
    linear_acc (2) = ( msg->samples[0].z + msg->samples[1].z + msg->samples[2].z )/3 ;

    angular_vel(0) = 0 ;
    angular_vel(1) = 0 ;
    angular_vel(2) = 0 ;
  }

  void estimateContactForce( Eigen::Vector3d & contact_forces ,
                             Eigen::Vector3d & contact_torques  )
  {

    // This is example 2.3 from PG 75 of
    // Optimal and Robust Estimation with an Introduction to Stochastic Control Theory, Second Edition by Lewis, F.L., et al; 2008

    kalmanForces.Update( Eigen::MatrixXd::Ones(1,1)*9, Eigen::MatrixXd::Zero(1,1));
//    contact_forces = kalmanForces.getStateEstimate();

    kalmanTorques.Update( Eigen::MatrixXd::Ones(1,1)*9, Eigen::MatrixXd::Zero(1,1));
//    contact_torques = kalmanTorques.getStateEstimate();

    msg.header.stamp.sec  = ros::Time::now().sec ;
    msg.header.stamp.nsec = ros::Time::now().nsec;

    Eigen::Quaterniond ft_to_acc(0.579, -0.406, -0.579, 0.406);

    Eigen::Vector3d transformed_force = ft_to_acc._transformVector( forces );

    msg.wrench.force.x = transformed_force ( 0 ) ;
    msg.wrench.force.y = transformed_force ( 1 ) ;
    msg.wrench.force.z = transformed_force ( 2 ) ;

    force_pub.publish( msg );

  }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Correctly transforms forces in ft link frame to acc frame
//rosrun tf tf_echo /r_gripper_motor_accelerometer_link /r_force_torque_link
//- Translation: [0.000, 0.000, 0.000]
//- Rotation: in Quaternion [-0.406, -0.579, 0.406, 0.579]
//            in RPY [-1.571, -0.349, 1.571]

//rosrun tf tf_echo /r_force_torque_link /r_gripper_motor_accelerometer_link
//- Translation: [0.000, 0.000, 0.000]
//- Rotation: in Quaternion [0.406, 0.579, -0.406, 0.579]
//            in RPY [3.140, 1.571, 0.000]

// Correctly transforms forces in ft link frame to acc frame
//rosrun tf tf_echo /l_gripper_motor_accelerometer_link /l_force_torque_link
//At time 1391047118.387
//- Translation: [0.000, 0.000, 0.000]
//- Rotation: in Quaternion [-0.406, -0.579, 0.406, 0.579]
//            in RPY [-1.571, -0.349, 1.571]

//rosrun tf tf_echo /l_force_torque_link /l_gripper_motor_accelerometer_link
//- Translation: [0.000, 0.000, 0.000]
//- Rotation: in Quaternion [0.406, 0.579, -0.406, 0.579]
//            in RPY [3.140, 1.571, 0.000]

PR2ForceEstimator f_est;

void forceCallback(const geometry_msgs::WrenchStamped::ConstPtr& msg)
{
  Eigen::Vector3d contact_forces ;
  Eigen::Vector3d contact_torques;

  f_est.updateForces( msg );
  f_est.estimateContactForce( contact_forces, contact_torques );
}

void accCallback(const pr2_msgs::AccelerometerState::ConstPtr& msg)
{
  f_est.updateAcc( msg );
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "interaction_force_estimator");

  ros::NodeHandle n;

  std::string r_ft_frame  = "r_force_torque_link";
  std::string r_acc_frame = "r_gripper_motor_accelerometer_link";

  ros::Subscriber f_sub  = n.subscribe( "/ft/r_gripper_motor"           , 1, forceCallback );
  ros::Subscriber acc_ub = n.subscribe( "/accelerometer/r_gripper_motor", 1, accCallback   );

  ros::spin();

  return 0;
}
