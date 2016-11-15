#include <ros/ros.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/String.h>

#include <Eigen/Core>
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
  int num_patches;
  int total_sensors;
  int data_idx;

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
	std::string para_num_patches = "/num_patches";

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
    if (!m_node.getParam( para_num_patches , num_patches))
    {
    	ROS_WARN("Parameter not found: %s", para_num_patches.c_str());
    	num_patches = 4;
    }

    ROS_INFO("Port:  %s",port.c_str());
    ROS_INFO("Baud:  %f",baud);
    ROS_INFO("Frame: %s",frame_id.c_str());
    ROS_INFO("Number of sensors: %d",num_sensors);
    ROS_INFO("Number of patches: %d",num_patches);

    total_sensors = num_sensors*num_patches;

	force		.resize(total_sensors);
	forceBias	.resize(total_sensors);
	force		.setZero();
	forceBias	.setOnes();

	pos			.resize(total_sensors,3);
	rot			.resize(total_sensors,4);
	pos			.setZero();
	rot			.setZero();

	if(num_sensors == 4 && num_patches == 1)
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

	if(num_sensors == 16 && num_patches == 1)
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

	if(num_sensors == 16 && num_patches == 8)
	{
		int idx = 0;
		double z   = 0.0;

		double r   = 0.05;                  // Radius of can
		double dt  = -2*M_PI/num_patches;   // Patch delta theta (rotate counter-clockwise)
		double p_l = 0.035;                 // Patch width
		double s_dx = p_l / 4.0;            // Sensor separation

		r = 3;
		p_l = sqrt(2*r*r*(1-cos(dt)));
		s_dx = p_l/4;

		for(int j=0;j<num_patches;j++)
		{
			double y = 3.0;	// TODO sqrt(num_sensors)-1; do not hardcode

			// Translation
			Eigen::Vector3f trans_vec(r*cos(j*dt),r*sin(j*dt),0);
			Eigen::Translation<float,3> trans(trans_vec);

			// Rotation of sensor plane
			Eigen::Matrix3f mat;
			mat =   Eigen::AngleAxisf(j*dt, Eigen::Vector3f::UnitZ())
			      * Eigen::AngleAxisf(0.5*M_PI, Eigen::Vector3f::UnitY());	// <- applied first
			Eigen::Transform<float,3,Eigen::Affine> t(mat);

			// Rotation such that x is the plane normal
			Eigen::Matrix3f mat2;
			mat2 =  Eigen::AngleAxisf(j*dt, Eigen::Vector3f::UnitZ());
			Eigen::Quaternionf q(mat2);

			for(int i=0;i<num_sensors;i++)
			{
				idx = senorIdx(j,i);

				// Sensor position vector
				Eigen::Vector3f vec;
				vec(0) = 3-i%4;     // 3,2,1,0,3,2,...
				vec(1) = y;         // 3,3,3,3,2,2,...
				vec(2) = z;

				if(i%4 == 3)
					y--;

				// Make 0,0,0 center of patch
				vec(0) -= 1.5;	// TODO 0.5*y
				vec(1) -= 1.5;

				// Scale with sensor separation
				vec *= s_dx;

				// Apply transforms
				vec = trans * t * vec;

				// Save results
				pos(idx,0) = vec(0);
				pos(idx,1) = vec(1);
				pos(idx,2) = vec(2);

				rot(idx,0) = q.w();
				rot(idx,1) = q.x();
				rot(idx,2) = q.y();
				rot(idx,3) = q.z();
			}
		}

		//std::cout<<"Position:\n"<<pos<<"\n---\n";
		//std::cout<<"Orientation:\n"<<rot<<"\n---\n";

	}

	firstRead=true;
	forceScale = 1023;

    // Flexiforce sensors
    tacSerial = new TactileSerial( port, baud, num_sensors, num_patches );
  }

  int senorIdx(int patch_idx, int sensor_idx)		// Patch: j=0..M-1, Sensors: i=0..N-1
  {
	  return (patch_idx*num_sensors+sensor_idx);	// Index: 0..N-1..2N-1..M*N-1
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

	  m_vizMarker.scale.y = 0.05;
	  m_vizMarker.scale.z = 0.05;

	  m_vizMarker.color.a = 1.0;

	  for(int i =0; i < total_sensors; i++)
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
		  {
			  m_vizMarker.scale.x = force(i);	// Arrow is always pointing along x
			  m_vizMarker.color.r = 1.0*   m_vizMarker.scale.x ;
			  m_vizMarker.color.g = 1.0*(1-m_vizMarker.scale.x);
			  m_vizMarker.color.b = 0.0;
		  }
		  else
		  {
			  m_vizMarker.color.r = 0.0;
			  m_vizMarker.color.g = 0.0;
			  m_vizMarker.color.b = 1.0;
			  m_vizMarker.scale.x = -0.02;
		  }

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
	  data_idx = 0;
	  if(!tacSerial->getDataArrayFromSerialPort( force, data_idx ))
	  {
		  std::cout<<"->Reading data failed!\n";
		  return;
	  }

	  //std::cout<<data_idx<<"\n";

	  if(data_idx == 31)	// TODO make sure all values have been updated
	  {
		  //force = (force - forceBias) * (2.0 / 1023.0);
		  force = force / forceScale;
		  publishRVizMarkerArray();
	  }
/*
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
*/
  }

  void measureBias()
  {
	  std::cout<<"#Estimating bias ...\n";

	  int patch_idx;
	  forceBias.setZero();
	  int loops = 0;
	  while(loops < 30)
	  {
		  if(tacSerial->getDataArrayFromSerialPort( force, patch_idx ))
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
	  //measureBias();
	  //forceBias *= 400.0;
	  while ( ros::ok() )
	  {
		  readAndPublish();
		  //ros::spinOnce();
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
