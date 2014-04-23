/*
 * controllerTestClient.cpp
 *
 *  Created on: Apr 23, 2014
 *      Author: isura
 */

#include <ros/ros.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <iostream>

using namespace std;

class controllerTestClient
{
  ros::NodeHandle  node;

  ros::ServiceClient m_switchControllerClient;
  ros::ServiceClient m_loadControllerClient;

  ros::Publisher m_rCartPub;
  ros::Publisher m_lCartPub;

  int choice;

/*
 * pr2_controller_manager/load_controller (pr2_mechanism_msgs/LoadController)
  pr2_controller_manager/unload_controller (pr2_mechanism_msgs/UnloadController)
  pr2_controller_manager/switch_controller (pr2_mechanism_msgs/SwitchController)
  pr2_controller_manager/reload_controller_libraries (pr2_mechanism_msgs/ReloadControllerLibraries)
  */

public:

  controllerTestClient()
  {
	  choice = 0;

	  m_switchControllerClient = node.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");
	  m_loadControllerClient   = node.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/load_controller");

	  m_rCartPub = node.advertise<geometry_msgs::PoseStamped>( "/r_cart/command_pose", 10 );
	  m_lCartPub = node.advertise<geometry_msgs::PoseStamped>( "/l_cart/command_pose", 10 );
  }

  ~controllerTestClient() { }

 void alignRightArm()
 {
	 geometry_msgs::PoseStamped pose;
	 pose.header.frame_id = "torso_lift_link";
	 pose.pose.position.x =  0.7 ;
	 pose.pose.position.y =  0.0 ;
	 pose.pose.position.z =  0.1 ;
	 pose.pose.orientation = tf::createQuaternionMsgFromYaw( 1.57079633 );

	 m_rCartPub.publish( pose );
 }

 void alignLeftArm()
 {
	 geometry_msgs::PoseStamped pose;
	 pose.header.frame_id = "torso_lift_link";
	 pose.pose.position.x =  0.7 ;
	 pose.pose.position.y =  0.5 ;
	 pose.pose.position.z = -0.1 ;
	 pose.pose.orientation = tf::createQuaternionMsgFromYaw( 1.57079633 );

	 m_lCartPub.publish( pose );
 }

void switchToCartneuroController()
{
	pr2_mechanism_msgs::SwitchController m_cartneuroControllerStart;
	m_cartneuroControllerStart.request.strictness  = m_cartneuroControllerStart.request.BEST_EFFORT;
	m_cartneuroControllerStart.request.start_controllers.push_back("pr2_cartneuroController");
	m_cartneuroControllerStart.request.stop_controllers.push_back("r_arm_controller");
	m_switchControllerClient.call(m_cartneuroControllerStart);
}

void switchToArmController()
{
	pr2_mechanism_msgs::SwitchController m_armControllerStart;
	m_armControllerStart.request.strictness  = m_armControllerStart.request.BEST_EFFORT;
	m_armControllerStart.request.start_controllers.push_back("r_arm_controller");
	m_armControllerStart.request.stop_controllers.push_back("pr2_cartneuroController");
	m_switchControllerClient.call(m_armControllerStart);
}

void switchLArmCartesianPoseController()
{
	pr2_mechanism_msgs::SwitchController m_armControllerStart;
	m_armControllerStart.request.strictness  = m_armControllerStart.request.BEST_EFFORT;
	m_armControllerStart.request.start_controllers.push_back("l_arm_cartesian_pose_controller");
	m_armControllerStart.request.stop_controllers.push_back("l_arm_controller");
	m_switchControllerClient.call(m_armControllerStart);
}

  void go()
  {
    ROS_INFO_STREAM("# Starting Experiment #");

    // Load the cart neuro controller
    pr2_mechanism_msgs::LoadController loadControllerCall;
    loadControllerCall.request.name = "PR2CartneuroControllerClass";
    m_loadControllerClient.call(loadControllerCall);

    while( ros::ok() )
    {

    	ROS_INFO_STREAM("Select your preference: ");
    	ROS_INFO_STREAM("1 - Align right arm");
    	ROS_INFO_STREAM("2 - Align left arm");
    	ROS_INFO_STREAM("3 - Switch to cartneuroController");
    	ROS_INFO_STREAM("4 - Switch to r_arm_controller");

    	std::cin >> choice ;

    	switch( choice )
    	{
    	    case 1 :
    	    		   alignRightArm();
					   break;
    	    case 2 :
    	    		   alignLeftArm();
					   break;
    	    case 3 :
    	    		   switchToCartneuroController();
					   break;
    	    case 4 :
    	    		   switchToArmController();
					   break;

    	    default :
    	    		   continue;
    	}

    }
  }

};

int
main( int argc, char** argv )
{
    // Initialize ROS
    ros::init (argc, argv, "controllerTestClient");
    ros::NodeHandle nh;

    controllerTestClient controllerTestClientObj;

    controllerTestClientObj.go();
}
