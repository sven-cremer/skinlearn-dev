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
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std;

class IntentEstimator
{
  ros::NodeHandle  m_node;
  ros::Subscriber  m_ftSub;
  ros::Publisher   m_ftVizPub;

  visualization_msgs::Marker      m_vizMarker;

  Eigen::Quaterniond m_ftFrame_to_accFrame ;
  Eigen::Vector3d    m_forceFT_accFrame    ;
  Eigen::Vector3d    m_forceFT_bias        ;

  bool m_firstTime;

public:

  IntentEstimator()
  {
	  m_ftSub    = m_node.subscribe("/ft/r_gripper_motor", 1000, &IntentEstimator::ftSensorCb, this);
	  m_ftVizPub = m_node.advertise<visualization_msgs::MarkerArray>("viz/r_ft", 1000);

	  //                                              w       x       y      z
	  m_ftFrame_to_accFrame = Eigen::Quaterniond( 0.579, -0.406, -0.579, 0.406 );
	  m_forceFT_accFrame    = Eigen::Vector3d::Zero();
	  m_forceFT_bias        = Eigen::Vector3d::Zero();

	  m_firstTime = true;
  }

  ~IntentEstimator() { }

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

	  forceFT = forceFT - m_forceFT_bias;

	  m_forceFT_accFrame = m_ftFrame_to_accFrame._transformVector( forceFT );

	  visualization_msgs::MarkerArray m_vizMarkerArray;

	  // X Axis
	  m_vizMarker.header.frame_id = "r_gripper_tool_frame";
	  m_vizMarker.header.stamp = ros::Time();
	  m_vizMarker.ns = "my_namespace";
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
	  m_vizMarker.pose.orientation.x = 0.0;
	  m_vizMarker.pose.orientation.y = 0.0;
	  m_vizMarker.pose.orientation.z = 0.0;
	  m_vizMarker.pose.orientation.w = 1.0;
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
	  m_vizMarker.pose.orientation.x = 0.0;
	  m_vizMarker.pose.orientation.y = 0.0;
	  m_vizMarker.pose.orientation.z = 0.0;
	  m_vizMarker.pose.orientation.w = 1.0;
	  m_vizMarker.scale.x = m_forceFT_accFrame.z()/forceScale;
	  m_vizMarker.scale.y = 0.01;
	  m_vizMarker.scale.z = 0.01;
	  m_vizMarker.color.a = 1.0;
	  m_vizMarker.color.r = 0.0;
	  m_vizMarker.color.g = 0.0;
	  m_vizMarker.color.b = 1.0;
	  m_vizMarkerArray.markers.push_back(m_vizMarker);

	  m_ftVizPub.publish(m_vizMarkerArray);

//	  ROS_INFO("Running");
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
