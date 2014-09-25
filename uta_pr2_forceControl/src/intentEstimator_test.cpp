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

  visualization_msgs::Marker    m_vizMarker;

  Eigen::Quaterniond m_ftFrame_to_accFrame ;
  Eigen::Vector3d    m_forceFT_accFrame    ;
  Eigen::Vector3d    m_forceFT_bias        ;

  Eigen::Vector3d    m_intentPos_p         ;
  Eigen::Vector3d    m_intentVel_p         ;
  Eigen::Vector3d    m_intentAcc_p         ;

  bool m_firstTime;

public:

  IntentEstimator()
  {
	  m_ftSub        = m_node.subscribe("/ft/r_gripper_motor", 1000, &IntentEstimator::ftSensorCb, this);
	  m_ftVizPub     = m_node.advertise<visualization_msgs::MarkerArray>("viz/r_ft", 1000);
	  m_intentVizPub = m_node.advertise<visualization_msgs::Marker>("viz/r_humanIntent", 1000);

	  //                                              w       x       y      z
	  m_ftFrame_to_accFrame = Eigen::Quaterniond( 0.579, -0.406, -0.579, 0.406 );
	  m_forceFT_accFrame    = Eigen::Vector3d::Zero();
	  m_forceFT_bias        = Eigen::Vector3d::Zero();

	  m_intentPos_p         = Eigen::Vector3d::Zero();
	  m_intentVel_p         = Eigen::Vector3d::Zero();
	  m_intentAcc_p         = Eigen::Vector3d::Zero();

	  m_firstTime = true;
  }

  ~IntentEstimator() { }

  void calcHumanIntentPos( Eigen::Vector3d & force, Eigen::Vector3d & pos )
  {
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

	  // ROS_INFO_STREAM(pos);
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

	  double forceScale = 10;

	  Eigen::Vector3d forceFT( msg->wrench.force.x,
                               msg->wrench.force.y,
                               msg->wrench.force.z );

	  // Reaction Force
	  forceFT = forceFT - m_forceFT_bias;

	  // Interaction force
	  forceFT = -forceFT;

	  m_forceFT_accFrame = m_ftFrame_to_accFrame._transformVector( forceFT );

	  visualization_msgs::MarkerArray m_vizMarkerArray;

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
	  m_vizMarker.scale.x = m_forceFT_accFrame.x()/forceScale;
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
	  m_vizMarker.scale.x = m_forceFT_accFrame.y()/forceScale;
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
	  m_vizMarker.scale.x = m_forceFT_accFrame.z()/forceScale;
	  m_vizMarker.scale.y = 0.01;
	  m_vizMarker.scale.z = 0.01;
	  m_vizMarker.color.a = 1.0;
	  m_vizMarker.color.r = 0.0;
	  m_vizMarker.color.g = 0.0;
	  m_vizMarker.color.b = 1.0;
	  m_vizMarkerArray.markers.push_back(m_vizMarker);

	  m_ftVizPub.publish(m_vizMarkerArray);

	  // Human intent calculation

	  Eigen::Vector3d pos;

	  calcHumanIntentPos( m_forceFT_accFrame, pos );

	  geometry_msgs::Point point0, point1;

	  point0.x = 0;
	  point0.y = 0;
	  point0.z = 0;

	  point1.x = pos.x();
	  point1.y = pos.y();
	  point1.z = pos.z();

	  visualization_msgs::Marker vizIntentMarker;

	  vizIntentMarker.header.frame_id = "/r_gripper_motor_accelerometer_link";
	  vizIntentMarker.header.stamp = ros::Time();
	  vizIntentMarker.ns = "vizIntent";
	  vizIntentMarker.id = 5;
	  vizIntentMarker.type = visualization_msgs::Marker::ARROW;
	  vizIntentMarker.action = visualization_msgs::Marker::ADD;

	  m_vizMarker.pose.position.x = 0;
	  m_vizMarker.pose.position.y = 0;
	  m_vizMarker.pose.position.z = 0;
	  m_vizMarker.pose.orientation.x = 0.0;
	  m_vizMarker.pose.orientation.y = 0.0;
	  m_vizMarker.pose.orientation.z = 0.0;
	  m_vizMarker.pose.orientation.w = 1.0;

	  vizIntentMarker.points.push_back(point0);
	  vizIntentMarker.points.push_back(point1);

	  vizIntentMarker.scale.x = 0.01;
	  vizIntentMarker.scale.y = 0.01;
	  vizIntentMarker.scale.z = 0.01;

	  vizIntentMarker.color.a = 1.0;
	  vizIntentMarker.color.r = 1.0;
	  vizIntentMarker.color.g = 0.0;
	  vizIntentMarker.color.b = 1.0;

	  m_intentVizPub.publish( vizIntentMarker );

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
