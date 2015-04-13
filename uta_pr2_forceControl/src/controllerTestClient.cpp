/*
 * controllerTestClient.cpp
 *
 *  Created on: Apr 23, 2014
 *      Author: isura
 */

#include <ros/ros.h>
#include <pr2_mechanism_msgs/LoadController.h>
#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/UnloadController.h>
#include <pr2_mechanism_msgs/ReloadControllerLibraries.h>
#include <pr2_mechanism_msgs/ListControllers.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/tf.h>
#include <iostream>
#include <std_srvs/Empty.h>
#include <neuroadaptive_msgs/fixedWeightToggle.h>
#include <string>

using namespace std;

class controllerTestClient
{
  ros::NodeHandle  m_node;

  ros::ServiceClient m_switchControllerClient   ;
  ros::ServiceClient m_loadControllerClient     ;
  ros::ServiceClient m_toggleFixedWeightsClient ;
  ros::ServiceClient m_unloadControllersClient  ;
  ros::ServiceClient m_reloadLibrariesClient    ;
  ros::ServiceClient m_listControllersClient    ;

  ros::Publisher     m_rCartPub                 ;
  ros::Publisher     m_lCartPub                 ;

  char choice ;

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

  double cartDes2X     ;
  double cartDes2Y     ;
  double cartDes2Z     ;
  double cartDes2Roll  ;
  double cartDes2Pitch ;
  double cartDes2Yaw   ;

  double cartDes3X     ;
  double cartDes3Y     ;
  double cartDes3Z     ;
  double cartDes3Roll  ;
  double cartDes3Pitch ;
  double cartDes3Yaw   ;

  double l_cartIniX     ;
  double l_cartIniY     ;
  double l_cartIniZ     ;
  double l_cartIniRoll  ;
  double l_cartIniPitch ;
  double l_cartIniYaw   ;

  bool useFixedWeights  ;

  double m_w0           ;
  double m_w1           ;
  double m_w2           ;
  double m_w3           ;
  double m_w4           ;
  double m_w5           ;
  double m_w6           ;
  double m_w7           ;

  std::vector<string> m_outerModel ;

  vector<string> m_unloadedControllers      ;
  vector<string> m_unloadedControllersState ;

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

	  /*
	  pr2_controller_manager/load_controller (pr2_mechanism_msgs/LoadController)
	  pr2_controller_manager/unload_controller (pr2_mechanism_msgs/UnloadController)
	  pr2_controller_manager/list_controllers (pr2_mechanism_msgs/ListControllers)
	  pr2_controller_manager/reload_controller_libraries (pr2_mechanism_msgs/ReloadControllerLibraries)
	  */

	  m_switchControllerClient   = m_node.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");
	  m_loadControllerClient     = m_node.serviceClient<pr2_mechanism_msgs::LoadController  >("pr2_controller_manager/load_controller");
	  m_toggleFixedWeightsClient = m_node.serviceClient<neuroadaptive_msgs::fixedWeightToggle>("pr2_cartneuroController/toggleFixedWeights");
	  m_unloadControllersClient  = m_node.serviceClient<pr2_mechanism_msgs::UnloadController>("pr2_controller_manager/unload_controller");
	  m_reloadLibrariesClient    = m_node.serviceClient<pr2_mechanism_msgs::ReloadControllerLibraries>("pr2_controller_manager/reload_controller_libraries");
	  m_listControllersClient    = m_node.serviceClient<pr2_mechanism_msgs::ListControllers>("pr2_controller_manager/list_controllers");
	  
	  m_rCartPub = m_node.advertise<geometry_msgs::PoseStamped>( "/l_cart/command_pose", 10 );
	  m_lCartPub = m_node.advertise<geometry_msgs::PoseStamped>( "/r_cart/command_pose", 10 );

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

	  if (!m_node.getParam( para_cartIniX     , r_cartIniX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniX.c_str())     ; }
	  if (!m_node.getParam( para_cartIniY     , r_cartIniY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniY.c_str())     ; }
	  if (!m_node.getParam( para_cartIniZ     , r_cartIniZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniZ.c_str())     ; }
	  if (!m_node.getParam( para_cartIniRoll  , r_cartIniRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniRoll .c_str()) ; }
	  if (!m_node.getParam( para_cartIniPitch , r_cartIniPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniPitch.c_str()) ; }
	  if (!m_node.getParam( para_cartIniYaw   , r_cartIniYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniYaw  .c_str()) ; }

	  std::string para_cartDesX     = "/cartDesX"      ;
	  std::string para_cartDesY     = "/cartDesY"      ;
	  std::string para_cartDesZ     = "/cartDesZ"      ;
	  std::string para_cartDesRoll  = "/cartDesRoll"   ;
	  std::string para_cartDesPitch = "/cartDesPitch"  ;
	  std::string para_cartDesYaw   = "/cartDesYaw"    ;

	  std::string para_cartDes2X     = "/cartDes2X"    ;
	  std::string para_cartDes2Y     = "/cartDes2Y"    ;
	  std::string para_cartDes2Z     = "/cartDes2Z"    ;
	  std::string para_cartDes2Roll  = "/cartDes2Roll" ;
	  std::string para_cartDes2Pitch = "/cartDes2Pitch";
	  std::string para_cartDes2Yaw   = "/cartDes2Yaw"  ;

	  std::string para_cartDes3X     = "/cartDes3X"    ;
	  std::string para_cartDes3Y     = "/cartDes3Y"    ;
	  std::string para_cartDes3Z     = "/cartDes3Z"    ;
	  std::string para_cartDes3Roll  = "/cartDes3Roll" ;
	  std::string para_cartDes3Pitch = "/cartDes3Pitch";
	  std::string para_cartDes3Yaw   = "/cartDes3Yaw"  ;

	  if (!m_node.getParam( para_cartDesX     , r_cartDesX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesX    .c_str()) ; }
	  if (!m_node.getParam( para_cartDesY     , r_cartDesY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesY    .c_str()) ; }
	  if (!m_node.getParam( para_cartDesZ     , r_cartDesZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesZ    .c_str()) ; }
	  if (!m_node.getParam( para_cartDesRoll  , r_cartDesRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesRoll .c_str()) ; }
	  if (!m_node.getParam( para_cartDesPitch , r_cartDesPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesPitch.c_str()) ; }
	  if (!m_node.getParam( para_cartDesYaw   , r_cartDesYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesYaw  .c_str()) ; }

	  if (!m_node.getParam( para_cartDes2X     , cartDes2X     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2X    .c_str()) ; }
	  if (!m_node.getParam( para_cartDes2Y     , cartDes2Y     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2Y    .c_str()) ; }
	  if (!m_node.getParam( para_cartDes2Z     , cartDes2Z     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2Z    .c_str()) ; }
	  if (!m_node.getParam( para_cartDes2Roll  , cartDes2Roll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2Roll .c_str()) ; }
	  if (!m_node.getParam( para_cartDes2Pitch , cartDes2Pitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2Pitch.c_str()) ; }
	  if (!m_node.getParam( para_cartDes2Yaw   , cartDes2Yaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2Yaw  .c_str()) ; }

	  if (!m_node.getParam( para_cartDes3X     , cartDes3X     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3X    .c_str()) ; }
	  if (!m_node.getParam( para_cartDes3Y     , cartDes3Y     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3Y    .c_str()) ; }
	  if (!m_node.getParam( para_cartDes3Z     , cartDes3Z     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3Z    .c_str()) ; }
	  if (!m_node.getParam( para_cartDes3Roll  , cartDes3Roll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3Roll .c_str()) ; }
	  if (!m_node.getParam( para_cartDes3Pitch , cartDes3Pitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3Pitch.c_str()) ; }
	  if (!m_node.getParam( para_cartDes3Yaw   , cartDes3Yaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3Yaw  .c_str()) ; }

	  std::string para_leftcartIniX     = "/lCartIniX";
	  std::string para_leftcartIniY     = "/lCartIniY";
	  std::string para_leftcartIniZ     = "/lCartIniZ";
	  std::string para_leftcartIniRoll  = "/lCartIniRoll";
	  std::string para_leftcartIniPitch = "/lCartIniPitch";
	  std::string para_leftcartIniYaw   = "/lCartIniYaw";

	  if (!m_node.getParam( para_leftcartIniX     , l_cartIniX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniX.c_str())     ; }
	  if (!m_node.getParam( para_leftcartIniY     , l_cartIniY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniY.c_str())     ; }
	  if (!m_node.getParam( para_leftcartIniZ     , l_cartIniZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniZ.c_str())     ; }
	  if (!m_node.getParam( para_leftcartIniRoll  , l_cartIniRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniRoll .c_str()) ; }
	  if (!m_node.getParam( para_leftcartIniPitch , l_cartIniPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniPitch.c_str()) ; }
	  if (!m_node.getParam( para_leftcartIniYaw   , l_cartIniYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_leftcartIniYaw  .c_str()) ; }

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

 void p3()
 {
	 geometry_msgs::PoseStamped pose;
	 pose.header.frame_id = "torso_lift_link";
	 pose.pose.position.x =  cartDes2X ;
	 pose.pose.position.y =  cartDes2Y ;
	 pose.pose.position.z =  cartDes2Z ;
	 pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( cartDes2Roll ,
                                                                      cartDes2Pitch,
                                                                      cartDes2Yaw    );
	 m_rCartPub.publish( pose );
 }

 void p4()
 {
	 geometry_msgs::PoseStamped pose;
	 pose.header.frame_id = "torso_lift_link";
	 pose.pose.position.x =  cartDes3X ;
 	 pose.pose.position.y =  cartDes3Y ;
 	 pose.pose.position.z =  cartDes3Z ;
 	 pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw( cartDes3Roll ,
 			                                                          cartDes3Pitch,
	 		                                                          cartDes3Yaw    );
	 m_rCartPub.publish( pose );
 }

void switchToCartneuroController()
{
	pr2_mechanism_msgs::SwitchController m_cartneuroControllerStart;
	m_cartneuroControllerStart.request.strictness  = m_cartneuroControllerStart.request.BEST_EFFORT;
	m_cartneuroControllerStart.request.start_controllers.push_back("pr2_cartneuroController");
	m_cartneuroControllerStart.request.stop_controllers.push_back("l_cart");
	m_switchControllerClient.call(m_cartneuroControllerStart);
}

void switchToArmController()
{
	pr2_mechanism_msgs::SwitchController m_armControllerStart;
	m_armControllerStart.request.strictness  = m_armControllerStart.request.BEST_EFFORT;
	m_armControllerStart.request.start_controllers.push_back("l_cart");
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

void toggleFixedWeights()
{
	neuroadaptive_msgs::fixedWeightToggle toggleSrv;
	m_toggleFixedWeightsClient.call(toggleSrv);
	useFixedWeights = toggleSrv.response.useFixedWeights;

	m_w0 = toggleSrv.response.w0 ;
	m_w1 = toggleSrv.response.w1 ;
	m_w2 = toggleSrv.response.w2 ;
	m_w3 = toggleSrv.response.w3 ;
	m_w4 = toggleSrv.response.w4 ;
	m_w5 = toggleSrv.response.w5 ;
	m_w6 = toggleSrv.response.w6 ;
	m_w7 = toggleSrv.response.w7 ;

	m_outerModel = toggleSrv.response.outerModel;
}

void unloadControllers()
{
	pr2_mechanism_msgs::SwitchController controllerSwitchSrv ;
	pr2_mechanism_msgs::ListControllers  listControllersSrv  ;
	pr2_mechanism_msgs::UnloadController unloadControllerSrv ;
	m_listControllersClient.call( listControllersSrv );

	for( uint i = 0; i < listControllersSrv.response.controllers.size(); i++ )
	{
		ROS_INFO_STREAM("Controller: " <<  listControllersSrv.response.controllers[i] << " | State: " << listControllersSrv.response.state[i] );
		if( listControllersSrv.response.state[i] == "running" )
		{
			ROS_DEBUG_STREAM("Need to stop!");
			controllerSwitchSrv.request.stop_controllers.push_back( listControllersSrv.response.controllers[i] );
		}
	}
	controllerSwitchSrv.request.strictness = controllerSwitchSrv.request.BEST_EFFORT;
	m_switchControllerClient.call( controllerSwitchSrv );

	for( uint i = 0; i < listControllersSrv.response.controllers.size(); i++ )
	{
		ROS_INFO_STREAM("Unloading: " << listControllersSrv.response.controllers[i] );
		unloadControllerSrv.request.name = listControllersSrv.response.controllers[i] ;
		m_unloadControllersClient.call(unloadControllerSrv);
	}

	// Save unloaded controllers
	m_unloadedControllers      = listControllersSrv.response.controllers ;
	m_unloadedControllersState = listControllersSrv.response.state       ;

}

void reloadLibraries()
{
	pr2_mechanism_msgs::ReloadControllerLibraries reloadLibSrv ;
	reloadLibSrv.request.force_kill = true ;
	m_reloadLibrariesClient.call(reloadLibSrv);
}

void reloadControllers()
{
	pr2_mechanism_msgs::LoadController loadControllerSrv ;
	for( uint i = 0; i < m_unloadedControllers.size(); i++ )
	{
		ROS_DEBUG_STREAM("Reload: " << m_unloadedControllers[i] );
		loadControllerSrv.request.name = m_unloadedControllers[i] ;
		m_loadControllerClient.call(loadControllerSrv);
	}

	pr2_mechanism_msgs::SwitchController controllerSwitchSrv ;
	for( uint i = 0; i < m_unloadedControllersState.size(); i++ )
	{
		// Only start previously running controllers
		if( m_unloadedControllersState[i] == "running" )
		{
			controllerSwitchSrv.request.start_controllers.push_back( m_unloadedControllers[i] ) ;
		}
	}
	controllerSwitchSrv.request.strictness = controllerSwitchSrv.request.BEST_EFFORT ;
	m_switchControllerClient.call( controllerSwitchSrv );

}

void reloadCartneuro()
{
	pr2_mechanism_msgs::SwitchController controllerSwitchSrv ;
	pr2_mechanism_msgs::ListControllers  listControllersSrv  ;
	pr2_mechanism_msgs::UnloadController unloadControllerSrv ;
	m_listControllersClient.call( listControllersSrv );

	std::string controllerName = "pr2_cartneuroController" ;

	controllerSwitchSrv.request.stop_controllers.push_back( controllerName );
	controllerSwitchSrv.request.strictness = controllerSwitchSrv.request.BEST_EFFORT;
	m_switchControllerClient.call( controllerSwitchSrv );

	ROS_INFO_STREAM("Unloading: " << controllerName )  ;
	unloadControllerSrv.request.name =  controllerName ;
	m_unloadControllersClient.call(unloadControllerSrv);

	pr2_mechanism_msgs::LoadController loadControllerSrv ;
	ROS_DEBUG_STREAM("Reload: " << controllerName ) ;
	loadControllerSrv.request.name = controllerName ;
	m_loadControllerClient.call(loadControllerSrv)  ;

	// Turn on neuro controller
//	pr2_mechanism_msgs::SwitchController controllerSwitchSrv ;
//	controllerSwitchSrv.request.start_controllers.push_back( controllerName ) ;
//	controllerSwitchSrv.request.strictness = controllerSwitchSrv.request.BEST_EFFORT ;
//	m_switchControllerClient.call( controllerSwitchSrv );
}

  void go()
  {
    ROS_INFO_STREAM("# Starting Experiment #");

    // Load the cart neuro controller
    pr2_mechanism_msgs::LoadController loadControllerCall;
    loadControllerCall.request.name = "pr2_cartneuroController";
    m_loadControllerClient.call(loadControllerCall);

    bool loopOn = true;
    while( ros::ok() && loopOn )
    {

    	ROS_INFO_STREAM("## Status ##");
    	ROS_INFO_STREAM("Toggle : " << useFixedWeights );
    	ROS_INFO_STREAM("Weights : " << m_w0 << " "
    	                             << m_w1 << " "
    	                             << m_w2 << " "
    	                             << m_w3 << " "
    	                             << m_w4 << " "
    	                             << m_w5 << " "
    	                             << m_w6 << " "
    	                             << m_w7 << " " );
    	vector<string>::iterator it1;
		for(it1 = m_outerModel.begin(); it1 != m_outerModel.end(); it1++)
		{
			ROS_INFO_STREAM( "Outer-loop : " << *it1 << " " );
		}
    	ROS_INFO_STREAM("Select your preference: ");
    	ROS_INFO_STREAM("1 - Align right arm");
    	ROS_INFO_STREAM("2 - Align left arm");
    	ROS_INFO_STREAM("3 - Switch to cartneuroController");
    	ROS_INFO_STREAM("4 - Switch to r_cart");
    	ROS_INFO_STREAM("5 - P1 (red)");
    	ROS_INFO_STREAM("6 - P2 (blue)");
    	ROS_INFO_STREAM("7 - P3 ( )");
    	ROS_INFO_STREAM("8 - P4 ( )");
    	ROS_INFO_STREAM("t - Toggle fixed weights");
    	ROS_INFO_STREAM("u - Unload all controllers");
    	ROS_INFO_STREAM("l - Reload libraries");
    	ROS_INFO_STREAM("r - Reload all controllers");
    	ROS_INFO_STREAM("n - Reload cartneuro");
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
    	    case '7' :
    	    		   p3();
					   break;
    	    case '8' :
    	    		   p4();
					   break;
    	    case 't' :
					   toggleFixedWeights();
					   break;
    	    case 'u' :
					   unloadControllers();
					   break;
    	    case 'l' :
					   reloadLibraries();
					   break;
    	    case 'r' :
					   reloadControllers();
					   break;
    	    case 'n' :
					   reloadCartneuro();
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
