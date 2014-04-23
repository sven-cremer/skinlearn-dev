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

  char choice;

  double r_cartIniX     ;
  double r_cartIniY     ;
  double r_cartIniZ     ;
  double r_cartIniRoll  ;
  double r_cartIniPitch ;
  double r_cartIniYaw   ;

  double r_cartDesX     ;
  double r_cartDesY     ;
  double r_cartDesZ     ;
  double r_cartDesRoll  ;
  double r_cartDesPitch ;
  double r_cartDesYaw   ;

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

	  r_cartDesX     = 0.7 ;
	  r_cartDesY     =-0.3 ;
	  r_cartDesZ     = 0.1 ;
	  r_cartDesRoll  = 0.0 ;
	  r_cartDesPitch = 0.0 ;
	  r_cartDesYaw   = 0.0 ;

	  l_cartIniX     = 0.7 ;
	  l_cartIniY     = 0.3 ;
	  l_cartIniZ     = 0.0 ;
	  l_cartIniRoll  = 0.0 ;
	  l_cartIniPitch = 0.0 ;
	  l_cartIniYaw   = -1.57079633 ;

	  std::string para_cartIniX     = "/cartIniX";
	  std::string para_cartIniY     = "/cartIniY";
	  std::string para_cartIniZ     = "/cartIniZ";
	  std::string para_cartIniRoll  = "/cartIniRoll";
	  std::string para_cartIniPitch = "/cartIniPitch";
	  std::string para_cartIniYaw   = "/cartIniYaw";

	  if (!node.getParam( para_cartIniX     , r_cartIniX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniX.c_str())     ; }
	  if (!node.getParam( para_cartIniY     , r_cartIniY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniY.c_str())     ; }
	  if (!node.getParam( para_cartIniZ     , r_cartIniZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniZ.c_str())     ; }
	  if (!node.getParam( para_cartIniRoll  , r_cartIniRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniRoll .c_str()) ; }
	  if (!node.getParam( para_cartIniPitch , r_cartIniPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniPitch.c_str()) ; }
	  if (!node.getParam( para_cartIniYaw   , r_cartIniYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniYaw  .c_str()) ; }

	  std::string para_cartDesX     = "/cartDesX";
	  std::string para_cartDesY     = "/cartDesY";
	  std::string para_cartDesZ     = "/cartDesZ";
	  std::string para_cartDesRoll  = "/cartDesRoll";
	  std::string para_cartDesPitch = "/cartDesPitch";
	  std::string para_cartDesYaw   = "/cartDesYaw";

	  if (!node.getParam( para_cartDesX     , r_cartDesX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesX.c_str())     ; }
	  if (!node.getParam( para_cartDesY     , r_cartDesY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesY.c_str())     ; }
	  if (!node.getParam( para_cartDesZ     , r_cartDesZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesZ.c_str())     ; }
	  if (!node.getParam( para_cartDesRoll  , r_cartDesRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesRoll .c_str()) ; }
	  if (!node.getParam( para_cartDesPitch , r_cartDesPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesPitch.c_str()) ; }
	  if (!node.getParam( para_cartDesYaw   , r_cartDesYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesYaw  .c_str()) ; }

	  std::string para_leftcartIniX     = "/lCartIniX";
	  std::string para_leftcartIniY     = "/lCartIniY";
	  std::string para_leftcartIniZ     = "/lCartIniZ";
	  std::string para_leftcartIniRoll  = "/lCartIniRoll";
	  std::string para_leftcartIniPitch = "/lCartIniPitch";
	  std::string para_leftcartIniYaw   = "/lCartIniYaw";

	  if (!node.getParam( para_leftcartIniX     , l_cartIniX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniX.c_str())     ; }
	  if (!node.getParam( para_leftcartIniY     , l_cartIniY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniY.c_str())     ; }
	  if (!node.getParam( para_leftcartIniZ     , l_cartIniZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniZ.c_str())     ; }
	  if (!node.getParam( para_leftcartIniRoll  , l_cartIniRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniRoll .c_str()) ; }
	  if (!node.getParam( para_leftcartIniPitch , l_cartIniPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniPitch.c_str()) ; }
	  if (!node.getParam( para_leftcartIniYaw   , l_cartIniYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniYaw  .c_str()) ; }

  }

  ~controllerTestClient() { }

 void alignRightArm()
 {
	 geometry_msgs::PoseStamped pose;
	 pose.header.frame_id = "torso_lift_link";
	 pose.pose.position.x =  r_cartIniX ;
	 pose.pose.position.y =  r_cartIniY ;
	 pose.pose.position.z =  r_cartIniZ ;
	 pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( r_cartIniRoll ,
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
 	 pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( l_cartIniRoll ,
 			                                                          l_cartIniPitch,
	 		                                                          l_cartIniYaw    );
	 m_lCartPub.publish( pose );
 }

 void p1()
 {
	 geometry_msgs::PoseStamped pose;
	 pose.header.frame_id = "torso_lift_link";
	 pose.pose.position.x =  r_cartIniX ;
	 pose.pose.position.y =  r_cartIniY ;
	 pose.pose.position.z =  r_cartIniZ ;
	 pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( r_cartIniRoll ,
                                                                      r_cartIniPitch,
                                                                      r_cartIniYaw    );
	 m_rCartPub.publish( pose );
 }

 void p2()
 {
	 geometry_msgs::PoseStamped pose;
	 pose.header.frame_id = "torso_lift_link";
	 pose.pose.position.x =  r_cartDesX ;
 	 pose.pose.position.y =  r_cartDesY ;
 	 pose.pose.position.z =  r_cartDesZ ;
 	 pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( r_cartDesRoll ,
 			                                                          r_cartDesPitch,
	 		                                                          r_cartDesYaw    );
	 m_rCartPub.publish( pose );
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
    	ROS_INFO_STREAM("5 - P1 (red)");
    	ROS_INFO_STREAM("6 - P2 (blue)");
    	ROS_INFO_STREAM("q - Quit");

    	std::cin >> choice ;

    	switch( choice )
    	{
    	    case '1' :
    	    		   alignRightArm();
					   break;
    	    case '2' :
    	    		   alignLeftArm();
					   break;
    	    case '3' :
    	    		   switchToCartneuroController();
					   break;
    	    case '4' :
    	    		   switchToArmController();
					   break;
    	    case '5' :
    	    		   p1();
					   break;
    	    case '6' :
    	    		   p2();
					   break;
    	    case 'q' :
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
