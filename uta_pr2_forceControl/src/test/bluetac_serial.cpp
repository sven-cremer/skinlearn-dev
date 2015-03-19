#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "uta_pr2_forceControl/tactile_serial.h"

#include <geometry_msgs/Wrench.h>

#include <vector>
#include <string>

#include <iostream>
#include <cstdio>

class TactileViz
{
  ros::NodeHandle  m_node;
  ros::Publisher   m_tactileVizPub;
  ros::Publisher   m_tactilePub;
  ros::Subscriber  m_sensorSub;

  geometry_msgs::Wrench m_flexiforce_wrench;

  bool firstRead;

  double forceScale;

  Eigen::VectorXd force;
  Eigen::VectorXd forceBias;
  Eigen::MatrixXd pos;

  visualization_msgs::Marker m_vizMarker;

  std::string port;
  double      baud;
  TactileSerial *tacSerial;

public:

  TactileViz( )
  {
	m_tactileVizPub = m_node.advertise<visualization_msgs::MarkerArray>("viz/tactile", 1);
	m_tactilePub    = m_node.advertise<geometry_msgs::Wrench>("pr2_cartneuroController/command", 1);

	force.resize(4);
	forceBias.resize(4);
	pos.resize(4,3);
	firstRead=true;

	pos << 0.1,-0.1, 0,
		  -0.1,-0.1, 0,
		  -0.1, 0.1, 0,
		   0.0, 0.0, 0;
		   //0.1, 0.1, 0;

	forceScale = 1024;

    std::string para_port = "/port";
    std::string para_baud = "/baud";

    if (!m_node.getParam( para_port , port )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_port.c_str()) ; return false; }
    if (!m_node.getParam( para_baud , baud )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_baud.c_str()) ; return false; }

    ROS_INFO_STREAM(port);
    ROS_INFO_STREAM(baud);

    // Flexiforce sensors
    tacSerial = new TactileSerial( port, baud );
  }

  TactileViz( int argc, char** argv )
  {
	m_tactileVizPub = m_node.advertise<visualization_msgs::MarkerArray>("viz/tactile", 1);
	m_tactilePub    = m_node.advertise<geometry_msgs::Wrench>("command", 1);
	force.resize(4);
	forceBias.resize(4);
	pos.resize(4,3);
	firstRead=true;

	pos << 0.1,-0.1, 0,
		  -0.1,-0.1, 0,
		  -0.1, 0.1, 0,
		   0.0, 0.0, 0;
		   //0.1, 0.1, 0;

	forceScale = 1024;

	port = "";
	baud = 0;

    // Flexiforce sensors
	tacSerial = new TactileSerial( argc, argv );
  }

  ~TactileViz() { }

  visualization_msgs::MarkerArray genVizvizMarkerArray( Eigen::MatrixXd & pos, Eigen::VectorXd & force )
  {
	  visualization_msgs::MarkerArray m_vizMarkerArray;

	  m_vizMarker.header.frame_id = "link";
	  m_vizMarker.header.stamp = ros::Time();
	  m_vizMarker.ns = "vizFt";
	  
	  m_vizMarker.type = visualization_msgs::Marker::ARROW;
	  m_vizMarker.action = visualization_msgs::Marker::ADD;

	  m_vizMarker.pose.orientation.x = 0.7071;
	  m_vizMarker.pose.orientation.y = 0.0;
	  m_vizMarker.pose.orientation.z = 0.7071;
	  m_vizMarker.pose.orientation.w = 0.0;

	for(int i =0; i < force.size(); i++)
	{
	  m_vizMarker.id = i;
	
	  m_vizMarker.pose.position.x = pos(i,0);
	  m_vizMarker.pose.position.y = pos(i,1);
	  m_vizMarker.pose.position.z = pos(i,2);

	  m_vizMarker.scale.x = force(i)/forceScale;
	  m_vizMarker.scale.y = 0.03;
	  m_vizMarker.scale.z = 0.03;

	  m_vizMarker.color.a = 1.0;
	  m_vizMarker.color.r = 1.0*   m_vizMarker.scale.x ;
	  m_vizMarker.color.g = 1.0*(1-m_vizMarker.scale.x);
	  m_vizMarker.color.b = 0.0;

	  m_vizMarkerArray.markers.push_back(m_vizMarker);
	 }

	  return m_vizMarkerArray;
  }

void publishTactileData()
  {
    tacSerial->getDataArrayFromSerialPort( force );
    force = force / 100.0;

	m_tactileVizPub.publish( genVizvizMarkerArray(pos, force ) );

	m_flexiforce_wrench.force.x  = force(0);
	m_flexiforce_wrench.force.y  = force(1);
	m_flexiforce_wrench.force.z  = force(2);
	m_flexiforce_wrench.torque.x = force(3);
	m_flexiforce_wrench.torque.y = 0.0;
	m_flexiforce_wrench.torque.z = 0.0;
	m_tactilePub.publish(m_flexiforce_wrench);
  }

int go()
{
  while ( ros::ok() )
  {
    publishTactileData();
    ros::spinOnce();
  }

  return 0;
}

};

int
main( int argc, char** argv )
{
    // Initialize ROS
    ros::init (argc, argv, "tactile_viz");
    ros::NodeHandle node;

    TactileViz tacViz;
//    TactileViz tacViz(argc, argv);

    tacViz.go();
}
