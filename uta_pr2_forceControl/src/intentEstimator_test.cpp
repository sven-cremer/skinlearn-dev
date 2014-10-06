/*
 * intentEstimator_test.cpp
 *
 *  Created on: Sep 24, 2014
 *      Author: sven
 */

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/WrenchStamped.h>
#include <geometry_msgs/Point.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

class IntentEstimator
{
  ros::NodeHandle  m_node;
  ros::Subscriber  m_ftSub;
  ros::Publisher   m_ftVizPub;
  ros::Publisher   m_intentVizPub;
  ros::Publisher   m_jerkPub;
  ros::Publisher   m_refTrajPub;
  ros::Publisher   m_X_m_pub;

  visualization_msgs::Marker    m_vizMarker;

  Eigen::Quaterniond m_ftFrame_to_accFrame ;
  Eigen::Vector3d    m_forceFT_accFrame    ;
  Eigen::Vector3d    m_forceFT_bias        ;

  Eigen::Vector3d    m_intentPos_p         ;
  Eigen::Vector3d    m_intentVel_p         ;
  Eigen::Vector3d    m_intentAcc_p         ;
  Eigen::Vector3d    m_intentFor_p         ;

  Eigen::Vector3d    m_cartDes             ;
  Eigen::Vector3d    m_cartIni             ;

  // Desired cartesian pose
  double m_cartDesX     ;
  double m_cartDesY     ;
  double m_cartDesZ     ;
  double m_cartDesRoll  ;
  double m_cartDesPitch ;
  double m_cartDesYaw   ;

  // Initial cartesian pose
  double m_cartIniX ;
  double m_cartIniY ;
  double m_cartIniZ ;

  bool m_firstTime;

public:

  IntentEstimator()
  {
	  m_ftSub        = m_node.subscribe("/ft/r_gripper_motor", 1000, &IntentEstimator::ftSensorCb, this);
	  m_ftVizPub     = m_node.advertise<visualization_msgs::MarkerArray>("viz/r_ft", 1000);
	  m_intentVizPub = m_node.advertise<visualization_msgs::Marker>("viz/r_humanIntent", 1000);
	  m_jerkPub      = m_node.advertise<geometry_msgs::Wrench>("force", 1000);
	  m_refTrajPub   = m_node.advertise<visualization_msgs::MarkerArray>("refTraj", 1000);
	  m_X_m_pub      = m_node.advertise<visualization_msgs::Marker>("x_m", 1000);

	  //                                              w       x       y      z
	  m_ftFrame_to_accFrame = Eigen::Quaterniond( 0.579, -0.406, -0.579, 0.406 );
	  m_forceFT_accFrame    = Eigen::Vector3d::Zero();
	  m_forceFT_bias        = Eigen::Vector3d::Zero();

	  m_intentPos_p         = Eigen::Vector3d::Zero();
	  m_intentVel_p         = Eigen::Vector3d::Zero();
	  m_intentAcc_p         = Eigen::Vector3d::Zero();

	  m_cartDes = Eigen::Vector3d( 0.7, -0.3, 0.1 );
	  m_cartIni = Eigen::Vector3d( 0.7,  0.0, 0.1 );

	  m_firstTime = true;
  }

  ~IntentEstimator() { }

  void calcHumanIntentPos( Eigen::Vector3d & force, Eigen::Vector3d & pos )
  {
//	  std::string para_cartDesX     = "/cartDesX";
//	  std::string para_cartDesY     = "/cartDesY";
//	  std::string para_cartDesZ     = "/cartDesZ";
//	  std::string para_cartDesRoll  = "/cartDesRoll";
//	  std::string para_cartDesPitch = "/cartDesPitch";
//	  std::string para_cartDesYaw   = "/cartDesYaw";
//
//
//	  std::string para_cartIniX     = "/cartIniX";
//	  std::string para_cartIniY     = "/cartIniY";
//	  std::string para_cartIniZ     = "/cartIniZ";
//
//	  if (!m_node.getParam( para_cartDesX     , m_cartDesX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesX    .c_str()) ; }
//	  if (!m_node.getParam( para_cartDesY     , m_cartDesY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesY    .c_str()) ; }
//	  if (!m_node.getParam( para_cartDesZ     , m_cartDesZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesZ    .c_str()) ; }
//	  if (!m_node.getParam( para_cartDesRoll  , m_cartDesRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesRoll .c_str()) ; }
//	  if (!m_node.getParam( para_cartDesPitch , m_cartDesPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesPitch.c_str()) ; }
//	  if (!m_node.getParam( para_cartDesYaw   , m_cartDesYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesYaw  .c_str()) ; }
//
//	  if (!m_node.getParam( para_cartIniX, m_cartIniX )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniX.c_str()) ; }
//	  if (!m_node.getParam( para_cartIniY, m_cartIniY )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniY.c_str()) ; }
//	  if (!m_node.getParam( para_cartIniZ, m_cartIniZ )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniZ.c_str()) ; }

	  Eigen::Vector3d intentPos = Eigen::Vector3d::Zero();
	  Eigen::Vector3d intentVel = Eigen::Vector3d::Zero();
	  Eigen::Vector3d intentAcc = Eigen::Vector3d::Zero();

	  Eigen::Vector3d M(1, 1, 1);

	  double delT = 0.1 ;

	  intentAcc = force.cwiseQuotient(M);

	  intentVel = intentVel + intentAcc * delT ;
	  intentPos = intentPos + intentVel * delT ;

	  m_intentPos_p = intentPos ;
	  m_intentVel_p = intentVel ;
	  m_intentAcc_p = intentAcc ;

	  pos = intentPos;

//	  ROS_INFO_STREAM(pos.transpose());
  }

  visualization_msgs::MarkerArray genVizvizMarkerArray( Eigen::Vector3d & ini, Eigen::Vector3d & des  )
  {
	  visualization_msgs::MarkerArray vizMarkerArray;
	  visualization_msgs::Marker      vizMarker;

	  // Ini
	  vizMarker.header.frame_id = "/torso_lift_link";
	  vizMarker.header.stamp = ros::Time();
	  vizMarker.ns = "refTraj";
	  vizMarker.id = 0;
	  vizMarker.type = visualization_msgs::Marker::CYLINDER;
	  vizMarker.action = visualization_msgs::Marker::ADD;
	  vizMarker.pose.position.x = ini.x();
	  vizMarker.pose.position.y = ini.y();
	  vizMarker.pose.position.z = ini.z();
	  vizMarker.pose.orientation.x = 0.0;
	  vizMarker.pose.orientation.y = 0.0;
	  vizMarker.pose.orientation.z = 0.0;
	  vizMarker.pose.orientation.w = 1.0;
	  vizMarker.scale.x = 0.02;
	  vizMarker.scale.y = 0.02;
	  vizMarker.scale.z = 0.05;
	  vizMarker.color.a = 1.0;
	  vizMarker.color.r = 1.0;
	  vizMarker.color.g = 0.0;
	  vizMarker.color.b = 0.0;
	  vizMarkerArray.markers.push_back(vizMarker);

	  // Des
	  vizMarker.id = 2;
	  vizMarker.pose.position.x = des.x();
	  vizMarker.pose.position.y = des.y();
	  vizMarker.pose.position.z = des.z();
	  vizMarker.color.a = 1.0;
	  vizMarker.color.r = 0.0;
	  vizMarker.color.g = 0.0;
	  vizMarker.color.b = 1.0;
	  vizMarkerArray.markers.push_back(vizMarker);

	  return vizMarkerArray;
  }

  visualization_msgs::MarkerArray genVizvizMarkerArray( Eigen::Vector3d & vector )
  {
	  visualization_msgs::MarkerArray m_vizMarkerArray;

	  double forceScale = 100;

	  // X Axis
	  m_vizMarker.header.frame_id = "/r_gripper_motor_accelerometer_link";
	  m_vizMarker.header.stamp = ros::Time();
	  m_vizMarker.ns = "vizFt";
	  m_vizMarker.id = 0;
	  m_vizMarker.type = visualization_msgs::Marker::ARROW;
	  m_vizMarker.action = visualization_msgs::Marker::ADD;
	  m_vizMarker.pose.position.x = 0;
	  m_vizMarker.pose.position.y = 0;
	  m_vizMarker.pose.position.z = 0;
	  m_vizMarker.pose.orientation.x = 0.0;
	  m_vizMarker.pose.orientation.y = 0.0;
	  m_vizMarker.pose.orientation.z = 0.0;
	  m_vizMarker.pose.orientation.w = 1.0;
	  m_vizMarker.scale.x = vector.x()/forceScale;
	  m_vizMarker.scale.y = 0.01;
	  m_vizMarker.scale.z = 0.01;
	  m_vizMarker.color.a = 1.0;
	  m_vizMarker.color.r = 1.0;
	  m_vizMarker.color.g = 0.0;
	  m_vizMarker.color.b = 0.0;
	  m_vizMarkerArray.markers.push_back(m_vizMarker);

	  // Y Axis
	  m_vizMarker.id = 1;
	  m_vizMarker.pose.orientation.x = 0.7071;
	  m_vizMarker.pose.orientation.y = 0.7071;
	  m_vizMarker.pose.orientation.z = 0.0;
	  m_vizMarker.pose.orientation.w = 0.0;
	  m_vizMarker.scale.x = vector.y()/forceScale;
	  m_vizMarker.scale.y = 0.01;
	  m_vizMarker.scale.z = 0.01;
	  m_vizMarker.color.a = 1.0;
	  m_vizMarker.color.r = 0.0;
	  m_vizMarker.color.g = 1.0;
	  m_vizMarker.color.b = 0.0;
	  m_vizMarkerArray.markers.push_back(m_vizMarker);

	  // Z Axis
	  m_vizMarker.id = 2;
	  m_vizMarker.pose.orientation.x = 0.7071;
	  m_vizMarker.pose.orientation.y = 0.0;
	  m_vizMarker.pose.orientation.z = 0.7071;
	  m_vizMarker.pose.orientation.w = 0.0;
	  m_vizMarker.scale.x = vector.z()/forceScale;
	  m_vizMarker.scale.y = 0.01;
	  m_vizMarker.scale.z = 0.01;
	  m_vizMarker.color.a = 1.0;
	  m_vizMarker.color.r = 0.0;
	  m_vizMarker.color.g = 0.0;
	  m_vizMarker.color.b = 1.0;
	  m_vizMarkerArray.markers.push_back(m_vizMarker);

	  return m_vizMarkerArray;
  }

  visualization_msgs::Marker genVizIntentMarker( geometry_msgs::Point point )
  {
	  visualization_msgs::Marker vizIntentMarker;

	  vizIntentMarker.header.frame_id = "/r_gripper_tool_frame";
	  vizIntentMarker.header.stamp = ros::Time();
	  vizIntentMarker.ns = "vizIntent";
	  vizIntentMarker.id = 5;
	  vizIntentMarker.type = visualization_msgs::Marker::ARROW;
	  vizIntentMarker.action = visualization_msgs::Marker::ADD;

	  vizIntentMarker.pose.position.x = 0;
	  vizIntentMarker.pose.position.y = 0;
	  vizIntentMarker.pose.position.z = 0;
	  vizIntentMarker.pose.orientation.x = 0.0;
	  vizIntentMarker.pose.orientation.y = 0.0;
	  vizIntentMarker.pose.orientation.z = 0.0;
	  vizIntentMarker.pose.orientation.w = 1.0;

	  geometry_msgs::Point point0;

	  point0.x = 0;
	  point0.y = 0;
	  point0.z = 0;

	  vizIntentMarker.points.push_back(point0);
	  vizIntentMarker.points.push_back(point);

	  vizIntentMarker.scale.x = 0.02;
	  vizIntentMarker.scale.y = 0.02;
	  vizIntentMarker.scale.z = 0.02;

	  vizIntentMarker.color.a = 1.0;
	  vizIntentMarker.color.r = 1.0;
	  vizIntentMarker.color.g = 1.0;
	  vizIntentMarker.color.b = 0.0;

	  return vizIntentMarker;
  }

  void ftSensorCb( geometry_msgs::WrenchStamped::Ptr msg )
  {
	  // Bias cancel
	  if( m_firstTime )
	  {
		  m_forceFT_bias = Eigen::Vector3d( msg->wrench.force.x,
                                            msg->wrench.force.y,
                                            msg->wrench.force.z );
		  m_firstTime = false;
	  }

	  Eigen::Vector3d forceFT( msg->wrench.force.x,
                               msg->wrench.force.y,
                               msg->wrench.force.z );

	  // Reaction Force
	  forceFT = forceFT - m_forceFT_bias;

	  // Interaction force
	  forceFT = -forceFT;

	  m_forceFT_accFrame = m_ftFrame_to_accFrame._transformVector( forceFT );

	  m_ftVizPub.publish(genVizvizMarkerArray( m_forceFT_accFrame ) );

	  // Human intent calculation

	  Eigen::Vector3d pos;

	  calcHumanIntentPos( m_forceFT_accFrame, pos );

	  geometry_msgs::Point point1;

	  point1.x = 0.0; //pos.x();
	  point1.y = pos.y();
	  point1.z = 0.0; //pos.z();


	  geometry_msgs::Wrench jerkMsg;

	  double delT = 0.001;
	  //Eigen::Vector3d Jerk = ( m_forceFT_accFrame - m_intentFor_p ) / delT;

	  jerkMsg.force.x = m_forceFT_accFrame.x();
	  jerkMsg.force.y = m_forceFT_accFrame.y();
	  jerkMsg.force.z = m_forceFT_accFrame.z();

	  m_intentFor_p = m_forceFT_accFrame;

//	  m_X_m_pub.publish

	  m_refTrajPub.publish( genVizvizMarkerArray(m_cartIni, m_cartDes ) );

	  m_intentVizPub.publish( genVizIntentMarker( point1 ) );
	  m_jerkPub.publish( jerkMsg );

  }

  void go()
  {
	  ros::spin();
  }

};

int
main( int argc, char** argv )
{
    // Initialize ROS
    ros::init (argc, argv, "intent_estimator_test");
    ros::NodeHandle node;

    IntentEstimator intentEstObj;

    intentEstObj.go();

}
