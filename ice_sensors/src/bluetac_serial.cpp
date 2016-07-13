#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "ice_sensors/tactile_serial.h"

#include <ice_msgs/tactileArrayData.h>
#include <geometry_msgs/WrenchStamped.h>

#include <vector>
#include <string>

#include <iostream>
#include <cstdio>

class TactileViz
{
  int num_sensors;

  ros::NodeHandle  m_node;
  ros::Publisher   m_tactileVizPub;
  ros::Publisher   m_tactileDataPub;
  ros::Publisher   m_tactileWrenchPub;

  ice_msgs::tactileArrayData   m_tactile_data;
  geometry_msgs::WrenchStamped m_tactile_wrench;

  bool firstRead;

  double forceScale;

  Eigen::VectorXd force;
  Eigen::VectorXd forceBias;
  Eigen::MatrixXd pos;
  Eigen::MatrixXd rot;

  std::string port;
  double      baud;

  std::string frame_id;

  TactileSerial *tacSerial;

public:

  TactileViz( )
  {
	m_tactileVizPub		= m_node.advertise<visualization_msgs::MarkerArray>("tactile/viz", 1);
	m_tactileDataPub    = m_node.advertise<ice_msgs::tactileArrayData>(		"tactile/data", 1);
	m_tactileWrenchPub  = m_node.advertise<geometry_msgs::WrenchStamped>(	"tactile/wrench", 1);

	// Read parameters
    std::string para_port = "/port";
    std::string para_baud = "/baud";
	std::string para_frame = "/tactile_frame_id";
	std::string para_num_sensors = "/num_sensors";

    if (!m_node.getParam( para_port , port ))
    {
    	ROS_WARN("Parameter not found: %s", para_port.c_str());
    	port = "/dev/ttyACM0";
    }
    if (!m_node.getParam( para_baud , baud ))
    {
    	ROS_WARN("Parameter not found: %s", para_baud.c_str());
    	baud = 2000000;
    }
    if (!m_node.getParam( para_frame , frame_id ))
    {
    	ROS_WARN("Parameter not found: %s", para_frame.c_str());
    	//frame_id = "/base_link";
    	frame_id = "/l_gripper_tool_frame";
    }
    if (!m_node.getParam( para_num_sensors , num_sensors ))
    {
    	ROS_WARN("Parameter not found: %s", para_num_sensors.c_str());
    	num_sensors = 4;
    }

    ROS_INFO("Port:  %s",port.c_str());
    ROS_INFO("Baud:  %f",baud);
    ROS_INFO("Frame: %s",frame_id.c_str());
    ROS_INFO("Number of sensors: %d",num_sensors);

	force		.resize(num_sensors);
	forceBias	.resize(num_sensors);
	force		.setZero();
	forceBias	.setZero();

	pos			.resize(num_sensors,3);
	rot			.resize(num_sensors,4);
	pos			.setZero();
	rot			.setZero();

	if(num_sensors == 4)
	{
		//      x,  y,  z,      direction
		pos <<  1,  0,  0,	 // -x
				0, -1,  0,   // +y
			   -1,  0,  0,   // +x
				0,  1,  0;   // -y
		pos = 0.1*pos;

		double r=0.70711;
		//     w, x, y, z,      direction
		rot << 0, 0, 0, 1,   // -x
			   r, 0, 0, r,   // +ya
			   1, 0, 0, 0,   // +x
			   r, 0, 0, -r;  // -y
	}

	if(num_sensors == 16)
	{
		//      x,  y,  z,
		pos <<  3,  3,  0,
				2,  3,  0,
				1,  3,  0,
				0,  3,  0,
				3,  2,  0,
				2,  2,  0,
				1,  2,  0,
				0,  2,  0,
				3,  1,  0,
				2,  1,  0,
				1,  1,  0,
				0,  1,  0,
				3,  0,  0,
				2,  0,  0,
				1,  0,  0,
				0,  0,  0;
		pos = 0.30*pos;
	}

	firstRead=true;
	forceScale = 1024;

    // Flexiforce sensors
    tacSerial = new TactileSerial( port, baud, num_sensors );
  }

  ~TactileViz() { }

  void publishRVizMarkerFlexiforce()
  {
	  visualization_msgs::MarkerArray m_vizMarkerArray;
	  visualization_msgs::Marker m_vizMarker;

	  m_vizMarker.header.frame_id = frame_id;
	  m_vizMarker.header.stamp = ros::Time::now();
	  m_vizMarker.ns = "vizFt";

	  m_vizMarker.type = visualization_msgs::Marker::ARROW;
	  m_vizMarker.action = visualization_msgs::Marker::ADD;

	  for(int i =0; i < num_sensors; i++)
	  {
		  m_vizMarker.id = i;

		  m_vizMarker.pose.position.x = pos(i,0);
		  m_vizMarker.pose.position.y = pos(i,1);
		  m_vizMarker.pose.position.z = pos(i,2);

		  m_vizMarker.pose.orientation.w = rot(i,0);
		  m_vizMarker.pose.orientation.x = rot(i,1);
		  m_vizMarker.pose.orientation.y = rot(i,2);
		  m_vizMarker.pose.orientation.z = rot(i,3);

		  if(force(i)>0.0)
			  m_vizMarker.scale.x = -force(i);	// Show reactant force
		  else
			  m_vizMarker.scale.x = -0.01;
		  m_vizMarker.scale.y = 0.02;
		  m_vizMarker.scale.z = 0.02;

		  m_vizMarker.color.a = 1.0;
		  m_vizMarker.color.r = 1.0*   m_vizMarker.scale.x ;
		  m_vizMarker.color.g = 1.0*(1-m_vizMarker.scale.x);
		  m_vizMarker.color.b = 0.0;

		  m_vizMarkerArray.markers.push_back(m_vizMarker);
	  }

	  m_tactileVizPub.publish( m_vizMarkerArray );
  }

  void publishRVizMarkerArray()
  {
	  visualization_msgs::MarkerArray m_vizMarkerArray;
	  visualization_msgs::Marker m_vizMarker;

	  m_vizMarker.header.frame_id = frame_id;
	  m_vizMarker.header.stamp = ros::Time::now();
	  m_vizMarker.ns = "vizFt";

	  m_vizMarker.type = visualization_msgs::Marker::ARROW;
	  m_vizMarker.action = visualization_msgs::Marker::ADD;

	  for(int i =0; i < num_sensors; i++)
	  {
		  m_vizMarker.id = i;

		  m_vizMarker.pose.position.x = pos(i,0);
		  m_vizMarker.pose.position.y = pos(i,1);
		  m_vizMarker.pose.position.z = pos(i,2);

		  m_vizMarker.pose.orientation.w = 0.70711;
		  m_vizMarker.pose.orientation.x = 0;
		  m_vizMarker.pose.orientation.y = 0.70711;
		  m_vizMarker.pose.orientation.z = 0;

		  if(force(i)>0.0)
			  m_vizMarker.scale.x = -force(i);	// Show reactant force
		  else
			  m_vizMarker.scale.x = -0.01;
		  m_vizMarker.scale.y = 0.02;
		  m_vizMarker.scale.z = 0.02;

		  m_vizMarker.color.a = 1.0;
		  m_vizMarker.color.r = 1.0*   m_vizMarker.scale.x ;
		  m_vizMarker.color.g = 1.0*(1-m_vizMarker.scale.x);
		  m_vizMarker.color.b = 0.0;

		  m_vizMarkerArray.markers.push_back(m_vizMarker);
	  }

	  m_tactileVizPub.publish( m_vizMarkerArray );
  }

  void publishTactileData()
  {
	  m_tactile_data.data.resize(num_sensors);
	  for( int i=0; i<num_sensors; i++)
	  {
		  m_tactile_data.data[i] = force(i);
	  }
	  m_tactileDataPub.publish(m_tactile_data);
  }

  void publishTactileWrench()
  {
	  // Tactile box layout
	  //
	  //             |
	  //      |------|------|
	  //      |             |
	  //      |      0      |
	  //
	  //      1  >   ^   >  3 +y
	  //
	  //             2
	  //            +x

	  // X-axis
	  if(force(0) > force(2))
	  {
		  m_tactile_wrench.wrench.force.x = -force(0);
	  }
	  else
	  {
		  m_tactile_wrench.wrench.force.x = force(2);
	  }
//	  m_tactile_wrench.wrench.force.x = force(2) - force(0);	// Issue: if one side is too sensitive it's always close to the max value
	  // Y-axis
	  if(force(1) > force(3))
	  {
		  m_tactile_wrench.wrench.force.y = force(1);
	  }
	  else
	  {
		  m_tactile_wrench.wrench.force.y = -force(3);
	  }
//	  m_tactile_wrench.wrench.force.y = force(1) - force(3);
	  // Z-axis
	  m_tactile_wrench.wrench.force.z = 0.0;

	  // Torque
	  m_tactile_wrench.wrench.torque.x = 0.0;
	  m_tactile_wrench.wrench.torque.y = 0.0;
	  m_tactile_wrench.wrench.torque.z = 0.0;

	  // Update header
	  m_tactile_wrench.header.frame_id = frame_id;
	  m_tactile_wrench.header.stamp = ros::Time::now();

	  // Publish topic
	  m_tactileWrenchPub.publish(m_tactile_wrench);
  }

  void readAndPublish()
  {
	  // Read data
	  if(!tacSerial->getDataArrayFromSerialPort( force ))
	  {
		  //std::cout<<"->Reading data failed!\n";
		  return;
	  }
	  // Remove bias
	  force = force - forceBias;
	  // Scale force
	  force = force / forceScale;
	  // Apply threshold
	  for( int i=0; i<num_sensors; i++)
	  {
		  if(force(i) < 0.035)	// TODO improve thresholding
		  {
			  force(i)=0;
		  }
	  }
	  //std::cout<<"Norm: "<<force.norm()<<"\n";

	  if(num_sensors == 4)
	  {
		  // Publish markers
		  publishRVizMarkerArray();

		  // Publish wrench
		  publishTactileWrench();

	  }
	  if(num_sensors == 16)
	  {
		  publishRVizMarkerArray();
	  }

	  // Publish data array
	  publishTactileData();
  }

  void measureBias()
  {
	  std::cout<<"#Estimating bias ...\n";

	  forceBias.setZero();
	  int loops = 0;
	  while(loops < 30)
	  {
		  if(tacSerial->getDataArrayFromSerialPort( force ))
		  {
			  forceBias += force;
			  loops++;
		  }
		  ros::Duration(0.1).sleep();
	  }
	  forceBias = forceBias / loops;
	  std::cout<<"Bias:\n"<<forceBias<<"\n---\n";
  }

  int go()
  {
	  measureBias();
	  while ( ros::ok() )
	  {
		  readAndPublish();
		  ros::spinOnce();
	  }
	  return 0;
  }

};

int
main( int argc, char** argv )
{
    // Initialize ROS
    ros::init (argc, argv, "tactile_node");
    ros::NodeHandle node;

    TactileViz tacViz;

    tacViz.go();
}
