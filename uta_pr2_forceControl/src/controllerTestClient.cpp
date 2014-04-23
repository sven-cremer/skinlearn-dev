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

  double r_cartIniX     ;
  double r_cartIniY     ;
  double r_cartIniZ     ;
  double r_cartIniRoll  ;
  double r_cartIniPitch ;
  double r_cartIniYaw   ;

  double l_cartIniX     ;
  double l_cartIniY     ;
  double l_cartIniZ     ;
  double l_cartIniRoll  ;
  double l_cartIniPitch ;
  double l_cartIniYaw   ;

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

	  // Initial cartesian pose
	  r_cartIniX     = 0.7 ;
	  r_cartIniY     =-0.3 ;
	  r_cartIniZ     = 0.1 ;
	  r_cartIniRoll  = 0.0 ;
	  r_cartIniPitch = 0.0 ;
	  r_cartIniYaw   = 0.0 ;

	  l_cartIniX     = 0.7 ;
	  l_cartIniY     = 0.3 ;
	  l_cartIniZ     = 0.0 ;
	  l_cartIniRoll  = 0.0 ;
	  l_cartIniPitch = 0.0 ;
	  l_cartIniYaw   = 0.0 ;

	  std::string para_cartIniX     = "/cartIniX";
	  std::string para_cartIniY     = "/cartIniY";
	  std::string para_cartIniZ     = "/cartIniZ";
	  std::string para_cartIniRoll  = "/cartIniRoll";
	  std::string para_cartIniPitch = "/cartIniPitch";
	  std::string para_cartIniYaw   = "/cartIniYaw";

	  if (!node.getParam( para_cartIniX     , r_cartIniX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniX.c_str()) ; return false; }
	  if (!node.getParam( para_cartIniY     , r_cartIniY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniY.c_str()) ; return false; }
	  if (!node.getParam( para_cartIniZ     , r_cartIniZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniZ.c_str()) ; return false; }
	  if (!node.getParam( para_cartIniRoll  , r_cartIniRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniRoll .c_str()) ; return false; }
	  if (!node.getParam( para_cartIniPitch , r_cartIniPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniPitch.c_str()) ; return false; }
	  if (!node.getParam( para_cartIniYaw   , r_cartIniYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniYaw  .c_str()) ; return false; }

	  std::string para_cartIniX     = "/leftcartIniX";
	  std::string para_cartIniY     = "/leftcartIniY";
	  std::string para_cartIniZ     = "/leftcartIniZ";
	  std::string para_cartIniRoll  = "/leftcartIniRoll";
	  std::string para_cartIniPitch = "/leftcartIniPitch";
	  std::string para_cartIniYaw   = "/leftcartIniYaw";

	  if (!node.getParam( para_cartIniX     , l_cartIniX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniX.c_str()) ; return false; }
	  if (!node.getParam( para_cartIniY     , l_cartIniY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniY.c_str()) ; return false; }
	  if (!node.getParam( para_cartIniZ     , l_cartIniZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniZ.c_str()) ; return false; }
	  if (!node.getParam( para_cartIniRoll  , l_cartIniRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniRoll .c_str()) ; return false; }
	  if (!node.getParam( para_cartIniPitch , l_cartIniPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniPitch.c_str()) ; return false; }
	  if (!node.getParam( para_cartIniYaw   , l_cartIniYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniYaw  .c_str()) ; return false; }


  }

  ~controllerTestClient() { }

 void alignRightArm()
 {
	 geometry_msgs::PoseStamped pose;
	 pose.header.frame_id = "torso_lift_link";
	 pose.pose.position.x =  r_cartIniX ;
	 pose.pose.position.y =  r_cartIniY ;
	 pose.pose.position.z =  r_cartIniZ ;
	 pose.pose.orientation = tf::createQuaternionFromRPY( r_cartIniRoll ,
			                                              r_cartIniPitch,
			                                              r_cartIniYaw    );
	 m_rCartPub.publish( pose );
 }

 void alignLeftArm()
 {
	 geometry_msgs::PoseStamped pose;
	 pose.header.frame_id = "torso_lift_link";
	 pose.pose.position.x =  l_cartIniX ;
 	 pose.pose.position.y =  l_cartIniY ;
 	 pose.pose.position.z =  l_cartIniZ ;
 	 pose.pose.orientation = tf::createQuaternionFromRPY( l_cartIniRoll ,
 			                                              l_cartIniPitch,
	 		                                              l_cartIniYaw    );
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
    // m_loadControllerClient.call(loadControllerCall);

    bool loopOn = true;
    while( ros::ok() && loopOn )
    {

    	ROS_INFO_STREAM("Select your preference: ");
    	ROS_INFO_STREAM("1 - Align right arm");
    	ROS_INFO_STREAM("2 - Align left arm");
    	ROS_INFO_STREAM("3 - Switch to cartneuroController");
    	ROS_INFO_STREAM("4 - Switch to r_arm_controller");
    	ROS_INFO_STREAM("5 - Quit");

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
    	    case 5 :
					   loopOn = false;
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
