/***********************************************************************************************************************
FILENAME:   pr2_cart_manager.h
AUTHORS:    Sven Cremer 		sven.cremer@mavs.uta.edu
            University of Texas at Arlington, Copyright (C) 2013.

DESCRIPTION:
Manger for pr2_cart realtime controller. Uses custom pr2_motion_clients packages for torso, arm, and gripper setup.

PUBLISHES:  NA
SUBSCRIBES: NA
SERVICES:   NA

REVISION HISTORY:
2014.02.07  SC     original file creation
2014.02.14  SC     code cleanup, new gripper client
2014.02.17  SC     services for setting gains
2015.10.26  SC     ported to hydro
***********************************************************************************************************************/

#ifndef PR2_CARTPULL_MANAGER_H_
#define PR2_CARTPULL_MANAGER_H_



#include <ros/ros.h>
#include <ros/package.h>

#include <pr2_mechanism_msgs/SwitchController.h>
#include <pr2_mechanism_msgs/ListControllers.h>

//#include <pr2_motion_clients/torsojointspacecontroller.h>
//#include <pr2_motion_clients/armjointspacecontroller.h>
//#include <pr2_motion_clients/gripper_both.h>

#include <pr2_controllers_msgs/JointTrajectoryAction.h>

// Service messages for changing gains
#include <ice_msgs/setValue.h>
#include <ice_msgs/setGains.h>
#include <ice_msgs/getState.h>

#include <apc_robot/apc_arms_joint.h>
#include <apc_robot/pr2_torso.h>
#include <apc_robot/apc_robot_grippers.h>

class PR2CartManager
{
public:

	  enum ControlState{
	    RUNNING,
	    STOPPED,
	    UNLOADED,
	    FAILURE
	  };

	PR2CartManager();
	~PR2CartManager();

	void robotInit(	bool open_grippers=true);
	void on(		bool close_grippers=true);
	void off(		bool open_grippers=true);

	void openGrippers();
	void closeGrippers();

	void printState();


	void switchControllers(const std::vector<std::string>& start_controllers, const std::vector<std::string>& stop_controllers);
	PR2CartManager::ControlState controllerState(std::string name);


	  bool set_Kd_rot_(double x, double y, double z) ;
	  bool set_Kd_vel_(double x, double y, double z) ;
	  bool set_Kp_rot_(double x, double y, double z) ;
	  bool set_Kp_vel_(double x, double y, double z) ;

	  bool set_restDGain_(double value)	;
	  bool set_restPGain_(double value) ;

	  bool set_psiThresh_(double value) ;
	  bool set_rThresh_(double value) 	;
	  bool set_rotDGain_(double value) 	;
	  bool set_rotPGain_(double value) 	;
	  bool set_velDGain_(double value) 	;
	  bool set_velPGain_(double value) 	;

	  bool get_State_(ice_msgs::getState * currentState);

	  void initGains();

private:

	ros::NodeHandle nh;
	ros::ServiceClient switch_controllers_service;
	ros::ServiceClient list_srv_;

	std::vector<std::string> arm_controllers_default;
	std::vector<std::string> arm_controllers_cart;

//	TorsoJointSpaceController torso;		// TODO: use apc_robot instead
//	ArmJointSpaceController arm;
	Gripper grippers;
	ArmsJoint arms;
	Torso torso;

	pr2_controllers_msgs::JointTrajectoryGoal leftArmStartPosition();
	pr2_controllers_msgs::JointTrajectoryGoal rightArmStartPosition();

	static std::string RIGHT_ARM_CONTROLLER;
	static std::string LEFT_ARM_CONTROLLER;
	static std::string PR2_CARTPULL_CONTROLLER;

	bool set_Value_(ros::ServiceClient * ptrSrvClient, double value);
	bool set_Gains_(ros::ServiceClient * ptrSrvClient, double K_x, double K_y, double K_z);

	ros::ServiceClient srv_set_Kd_rot;
	ros::ServiceClient srv_set_Kd_vel;
	ros::ServiceClient srv_set_Kp_rot;
	ros::ServiceClient srv_set_Kp_vel;

	ros::ServiceClient srv_set_restDGain;
	ros::ServiceClient srv_set_restPGain;

	ros::ServiceClient srv_set_psiThresh;
	ros::ServiceClient srv_set_rThresh;
	ros::ServiceClient srv_set_rotDGain;
	ros::ServiceClient srv_set_rotPGain;
	ros::ServiceClient srv_set_velDGain;
	ros::ServiceClient srv_set_velPGain;
	ros::ServiceClient srv_get_State;

	ros::ServiceClient srv_reinitCtrl;

};





#endif /* PR2_CARTPULL_MANAGER_H_ */
