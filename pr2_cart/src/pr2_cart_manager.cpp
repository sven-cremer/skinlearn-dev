/***********************************************************************************************************************
FILENAME:   pr2_cart_manager.cpp
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
***********************************************************************************************************************/

#include <pr2_cart/pr2_cart_manager.h>

/***********************************************************************************************************************
controllers
 ***********************************************************************************************************************/
std::string PR2CartManager::RIGHT_ARM_CONTROLLER  	= "r_arm_controller";
std::string PR2CartManager::LEFT_ARM_CONTROLLER   	= "l_arm_controller";
std::string PR2CartManager::PR2_CARTPULL_CONTROLLER = "pr2_cart";

/***********************************************************************************************************************
initializes services and clients
***********************************************************************************************************************/
PR2CartManager::PR2CartManager()
{
	list_srv_ = nh.serviceClient<pr2_mechanism_msgs::ListControllers>("pr2_controller_manager/list_controllers");

	// Switch client for cart pushing
	switch_controllers_service = nh.serviceClient<pr2_mechanism_msgs::SwitchController>("pr2_controller_manager/switch_controller");
	arm_controllers_default.push_back(LEFT_ARM_CONTROLLER);
	arm_controllers_default.push_back(RIGHT_ARM_CONTROLLER);
	arm_controllers_cart.push_back(PR2_CARTPULL_CONTROLLER);


	// Initialize the current Controller modes                  // TODO: make this a function (update current controllers)
	if(controllerState(LEFT_ARM_CONTROLLER) == RUNNING)
	{
		ROS_INFO("Left arm in POSITION_CONTROL");
	}
	else if(controllerState(LEFT_ARM_CONTROLLER) == STOPPED)
	{
		ROS_WARN("Turning on arm controllers ...");
		switchControllers(arm_controllers_default,arm_controllers_cart);
	}
	else
	{
		ROS_ERROR("No left arm controller running!");
		exit(-1);
	}

	if(controllerState(RIGHT_ARM_CONTROLLER) == RUNNING)
	{
		ROS_INFO("Right arm in POSITION_CONTROL");
	}
	else if(controllerState(RIGHT_ARM_CONTROLLER) == STOPPED)
	{
		ROS_WARN("Turning on arm controllers ...");
		switchControllers(arm_controllers_default,arm_controllers_cart);
	}
	else
	{
		ROS_ERROR("No right arm controller running!");
		exit(-1);
	}

	if(controllerState(PR2_CARTPULL_CONTROLLER) == RUNNING)
	{
		ROS_WARN("Turning off cart controller");
		switchControllers(arm_controllers_default,arm_controllers_cart);
	}
	else if(controllerState(PR2_CARTPULL_CONTROLLER) == STOPPED)
	{
		ROS_INFO("Cart controller is stopped ...");
	}
	else
	{
		ROS_ERROR("No cart controller running!");
		exit(-1);
	}


	  // Wait for services
	  ROS_INFO("Waiting for /pr2_cart services...");

	  ros::service::waitForService("/pr2_cart/set_Kd_rot",-1);
	  ros::service::waitForService("/pr2_cart/set_Kd_vel",-1);
	  ros::service::waitForService("/pr2_cart/set_Kp_rot",-1);
	  ros::service::waitForService("/pr2_cart/set_Kp_vel",-1);
	  ros::service::waitForService("/pr2_cart/set_psiThresh",-1);
	  ros::service::waitForService("/pr2_cart/set_rThresh",-1);
	  ros::service::waitForService("/pr2_cart/set_restDGain",-1);
	  ros::service::waitForService("/pr2_cart/set_restPGain",-1);
	  ros::service::waitForService("/pr2_cart/set_rotDGain",-1);
	  ros::service::waitForService("/pr2_cart/set_rotPGain",-1);
	  ros::service::waitForService("/pr2_cart/set_velDGain",-1);
	  ros::service::waitForService("/pr2_cart/set_velPGain",-1);
	  ros::service::waitForService("/pr2_cart/getState",-1);

	  srv_set_Kd_rot = nh.serviceClient<ice_msgs::setGains>("/pr2_cart/set_Kd_rot");
	  srv_set_Kd_vel = nh.serviceClient<ice_msgs::setGains>("/pr2_cart/set_Kd_vel");
	  srv_set_Kp_rot = nh.serviceClient<ice_msgs::setGains>("/pr2_cart/set_Kp_rot");
	  srv_set_Kp_vel = nh.serviceClient<ice_msgs::setGains>("/pr2_cart/set_Kp_vel");

	  srv_set_restDGain	= nh.serviceClient<ice_msgs::setValue>("/pr2_cart/set_restDGain");
	  srv_set_restPGain = nh.serviceClient<ice_msgs::setValue>("/pr2_cart/set_restPGain");

	  srv_set_psiThresh = nh.serviceClient<ice_msgs::setValue>("/pr2_cart/set_psiThresh");
	  srv_set_rThresh 	= nh.serviceClient<ice_msgs::setValue>("/pr2_cart/set_rThresh");
	  srv_set_rotDGain 	= nh.serviceClient<ice_msgs::setValue>("/pr2_cart/set_rotDGain");
	  srv_set_rotPGain 	= nh.serviceClient<ice_msgs::setValue>("/pr2_cart/set_rotPGain");
	  srv_set_velDGain 	= nh.serviceClient<ice_msgs::setValue>("/pr2_cart/set_velDGain");
	  srv_set_velPGain 	= nh.serviceClient<ice_msgs::setValue>("/pr2_cart/set_velPGain");

	  srv_get_State = nh.serviceClient<ice_msgs::getState>("/pr2_cart/getState");


//	  ros::service::waitForService("/pr2_cart/reinitCtrl",-1);
//	  srv_reinitCtrl = nh.serviceClient<ice_msgs::setValue>("/pr2_cart/reinitCtrl");

		// Position robot with grippers open
		robotInit(true);


	ROS_INFO("PR2CartManager initialized!");
}
/***********************************************************************************************************************
Initialize robot position
***********************************************************************************************************************/
void PR2CartManager::robotInit(bool open_grippers)
{
	off();	//Make sure manager is off

	// Open grippers
//	if(open_grippers)
//		grippers.open();

	// Lift torso
//	torso.position(0.05);					//To avoid waiting: torso.sendGoal(0.3);    // TODO make this a variable (use 0.3 with cart)
//	ROS_INFO("Torso at %f meters.", 0.01);

	// Position arms
//	arm.sendGoal(leftArmStartPosition());
//	arm.sendGoal(rightArmStartPosition());

//	while(!arm.motionComplete()||!grippers.motionComplete())					// TODO: don't move the arms while the grippers are close
//	{																		//       this could be very bad if the PR2 is grabbing onto something
//		ROS_INFO("Waiting for arm and/or gripper motions to complete ...");
//		sleep(2);
//	}

	torso.sendGoal(0.2);


	std::vector<double> l_joints;
	l_joints.push_back(0.149233);
	l_joints.push_back(1.16291);
	l_joints.push_back(0.159471);
	l_joints.push_back(-1.71565);
	l_joints.push_back(-3.1908);
	l_joints.push_back(-0.521468);
	l_joints.push_back(-1.52892);

	std::vector<double> r_joints;
	r_joints.push_back(0.0063641);
	r_joints.push_back(1.1557);
	r_joints.push_back(-0.00750675);
	r_joints.push_back(-1.73534);
	r_joints.push_back(3.09916);
	r_joints.push_back(-0.607375);
	r_joints.push_back(-1.5531);

	arms.sendGoal(l_joints, ArmsJoint::LEFT);
	arms.sendGoal(r_joints, ArmsJoint::RIGHT);

	ROS_INFO("PR2 in position!");

	//Re-initialize controller
	//set_Value_(&srv_reinitCtrl, 0);  TODO do this only when running?

}
/***********************************************************************************************************************
Turn on
***********************************************************************************************************************/
void PR2CartManager::on(bool close_grippers)
{
//	if(close_grippers)
//		closeGrippers();
	switchControllers(arm_controllers_cart, arm_controllers_default);
}
/***********************************************************************************************************************
Turn off
***********************************************************************************************************************/
void PR2CartManager::off(bool open_grippers)
{
	switchControllers(arm_controllers_default, arm_controllers_cart);
//	if(open_grippers)
//		openGrippers();
}
/***********************************************************************************************************************
Open/close grippers
***********************************************************************************************************************/
void PR2CartManager::openGrippers()
{
	grippers.open();
	while(!grippers.motionComplete())
	{
		ROS_INFO("Waiting for gripper motions to complete ...");
		sleep(2);
	}
}
void PR2CartManager::closeGrippers()
{
	grippers.close();
	while(!grippers.motionComplete())
	{
		ROS_INFO("Waiting for gripper motions to complete ...");
		sleep(2);
	}
}
/***********************************************************************************************************************
Set gain values
***********************************************************************************************************************/
bool PR2CartManager::set_Value_(ros::ServiceClient * ptrSrvClient, double value)
{
	ice_msgs::setValue srv_setValue;
	srv_setValue.request.value = value;
	if (ptrSrvClient->call(srv_setValue))
	{
		ROS_INFO("Service request sent!");
		return true;
	}
	else
	{
		ROS_ERROR("Failed to call service!");
		return false;
	}
}
bool PR2CartManager::set_Gains_(ros::ServiceClient * ptrSrvClient, double K_x, double K_y, double K_z)
{
	ice_msgs::setGains srv_setGains;
	srv_setGains.request.K_x = K_x;
	srv_setGains.request.K_y = K_y;
	srv_setGains.request.K_z = K_z;
	if (ptrSrvClient->call(srv_setGains))
	{
		ROS_INFO("Service request sent!");
		return true;
	}
	else
	{
		ROS_ERROR("Failed to call service!");
		return false;
	}
}

bool PR2CartManager::set_restDGain_(double value)	{return set_Value_(&srv_set_restDGain, value);}
bool PR2CartManager::set_restPGain_(double value) 	{return set_Value_(&srv_set_restPGain, value);}

bool PR2CartManager::set_psiThresh_(double value) 	{return set_Value_(&srv_set_psiThresh, value);}
bool PR2CartManager::set_rThresh_(double value) 	{return set_Value_(&srv_set_rThresh, value);}
bool PR2CartManager::set_rotDGain_(double value) 	{return set_Value_(&srv_set_rotDGain, value);}
bool PR2CartManager::set_rotPGain_(double value) 	{return set_Value_(&srv_set_rotPGain, value);}
bool PR2CartManager::set_velDGain_(double value) 	{return set_Value_(&srv_set_velDGain, value);}
bool PR2CartManager::set_velPGain_(double value) 	{return set_Value_(&srv_set_velPGain, value);}

bool PR2CartManager::set_Kd_rot_(double x, double y, double z) {return set_Gains_(&srv_set_Kd_rot, x, y, z);}
bool PR2CartManager::set_Kd_vel_(double x, double y, double z) {return set_Gains_(&srv_set_Kd_vel, x, y, z);}
bool PR2CartManager::set_Kp_rot_(double x, double y, double z) {return set_Gains_(&srv_set_Kp_rot, x, y, z);}
bool PR2CartManager::set_Kp_vel_(double x, double y, double z) {return set_Gains_(&srv_set_Kp_vel, x, y, z);}

void PR2CartManager::initGains()
{
	set_restDGain_(0.0);
	set_restPGain_(3.5);

	set_rotDGain_(0.0);
	set_rotPGain_(3.5);
	set_velDGain_(0.0);
	set_velPGain_(3.5);

	set_psiThresh_(0.10);
	set_rThresh_(0.05);

	set_Kp_vel_(100.0,100.0,100.0);
	set_Kp_rot_(100.0,100.0,10.0);

	set_Kd_vel_(1.0,1.0,1.0);
	set_Kd_rot_(1.0,1.0,1.0);
}

bool PR2CartManager::get_State_(ice_msgs::getState * currentState)
{
	ice_msgs::getState temp;

	if (srv_get_State.call(temp))
	{
		currentState->response = temp.response;
		return true;
	}
	{
		ROS_ERROR("Failed to get state!");
		return false;
	}
}

/***********************************************************************************************************************
Print state
***********************************************************************************************************************/
void PR2CartManager::printState()
{
	ice_msgs::getState srv_getState;

	if (srv_get_State.call(srv_getState))
	{
		ROS_INFO_STREAM("Current state: " <<
				srv_getState.response.restPGain << ", " <<
				srv_getState.response.restDGain << ", " <<
				srv_getState.response.velPGain << ", " <<
				srv_getState.response.velDGain << ", " <<
				srv_getState.response.rotPGain << ", " <<
				srv_getState.response.rotDGain << ", " <<
				srv_getState.response.rThresh << ", " <<
				srv_getState.response.psiThresh << ", " <<
				srv_getState.response.Kp_vel_x << ", " <<
				srv_getState.response.Kp_vel_y << ", " <<
				srv_getState.response.Kp_vel_z << ", " <<
				srv_getState.response.Kd_vel_x << ", " <<
				srv_getState.response.Kd_vel_y << ", " <<
				srv_getState.response.Kd_vel_z << ", " <<
				srv_getState.response.Kp_rot_x << ", " <<
				srv_getState.response.Kp_rot_y << ", " <<
				srv_getState.response.Kp_rot_z << ", " <<
				srv_getState.response.Kd_rot_x << ", " <<
				srv_getState.response.Kd_rot_y << ", " <<
				srv_getState.response.Kd_rot_z);
	}
	else
	{
		ROS_ERROR("Failed to get state!");
	}

}

/***********************************************************************************************************************
clean up clients
***********************************************************************************************************************/
PR2CartManager::~PR2CartManager()
{
	//off;

}
/***********************************************************************************************************************
Goal objects for starting positions
***********************************************************************************************************************/
pr2_controllers_msgs::JointTrajectoryGoal PR2CartManager::leftArmStartPosition()
{

	pr2_controllers_msgs::JointTrajectoryGoal goal_left;

	double l_start[7] = {0.149233, 1.16291, 0.159471, -1.71565, -3.1908, -0.521468, -1.52892};

	goal_left.trajectory.joint_names.push_back("l_shoulder_pan_joint");
	goal_left.trajectory.joint_names.push_back("l_shoulder_lift_joint");
	goal_left.trajectory.joint_names.push_back("l_upper_arm_roll_joint");
	goal_left.trajectory.joint_names.push_back("l_elbow_flex_joint");
	goal_left.trajectory.joint_names.push_back("l_forearm_roll_joint");
	goal_left.trajectory.joint_names.push_back("l_wrist_flex_joint");
	goal_left.trajectory.joint_names.push_back("l_wrist_roll_joint");

	// Only one waypoint
	goal_left.trajectory.points.resize(1);

	goal_left.trajectory.points[0].positions.resize(7);
	goal_left.trajectory.points[0].velocities.resize(7);

	for (int i = 0; i<7; ++i)
	{
		goal_left.trajectory.points[0].positions[i] = l_start[i];
		goal_left.trajectory.points[0].velocities[i] = 0.0;
	}

	// To be reached 3 second after starting along the trajectory
	goal_left.trajectory.points[0].time_from_start = ros::Duration(3.0);

	goal_left.trajectory.header.stamp = ros::Time::now();

	return goal_left;
}

pr2_controllers_msgs::JointTrajectoryGoal PR2CartManager::rightArmStartPosition()
{
	  pr2_controllers_msgs::JointTrajectoryGoal goal_right;

	  double r_start[7] = {0.0063641, 1.1557, -0.00750675, -1.73534, 3.09916, -0.607375, -1.5531};

	  goal_right.trajectory.joint_names.push_back("r_shoulder_pan_joint");
	  goal_right.trajectory.joint_names.push_back("r_shoulder_lift_joint");
	  goal_right.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
	  goal_right.trajectory.joint_names.push_back("r_elbow_flex_joint");
	  goal_right.trajectory.joint_names.push_back("r_forearm_roll_joint");
	  goal_right.trajectory.joint_names.push_back("r_wrist_flex_joint");
	  goal_right.trajectory.joint_names.push_back("r_wrist_roll_joint");

	  // Only one waypoint
	  goal_right.trajectory.points.resize(1);

	  goal_right.trajectory.points[0].positions.resize(7);
	  goal_right.trajectory.points[0].velocities.resize(7);

	  for (int i = 0; i<7; ++i)
	  {
		  goal_right.trajectory.points[0].positions[i] = r_start[i];
		  goal_right.trajectory.points[0].velocities[i] = 0.0;
	  }

	  // To be reached 3 second after starting along the trajectory
	  goal_right.trajectory.points[0].time_from_start = ros::Duration(3.0);

	  goal_right.trajectory.header.stamp = ros::Time::now();

	  return goal_right;
}
/***********************************************************************************************************************
Get current state of controller
example: http://docs.ros.org/hydro/api/pr2_controller_manager/html/test_8cpp_source.html
 ***********************************************************************************************************************/
PR2CartManager::ControlState PR2CartManager::controllerState(std::string name)
{
	pr2_mechanism_msgs::ListControllers srv_msg;
	if (!list_srv_.call(srv_msg))
	{
		ROS_WARN("Failed to call list controller service!");
		return FAILURE;
	}
	for (unsigned int i=0; i<srv_msg.response.controllers.size(); i++)
	{
		if (name == srv_msg.response.controllers[i])
		{
			//ROS_INFO(" %s : %s", srv_msg.response.controllers[i].c_str(), srv_msg.response.state[i].c_str());
			if (srv_msg.response.state[i] == "running") return RUNNING;
			else if (srv_msg.response.state[i] == "stopped") return STOPPED;
			else return FAILURE;
		}
	}
	ROS_WARN("Controller not listed!");
	return UNLOADED;
}
/***********************************************************************************************************************
switch controllers
***********************************************************************************************************************/
void PR2CartManager::switchControllers(const std::vector<std::string>& start_controllers, const std::vector<std::string>& stop_controllers) {

	pr2_mechanism_msgs::SwitchController::Request req;
	pr2_mechanism_msgs::SwitchController::Response res;
	req.start_controllers = start_controllers;
	req.stop_controllers = stop_controllers;
	for(std::vector<std::string>::const_iterator it = start_controllers.begin();
			it != start_controllers.end();
			it++)
	{
		ROS_INFO_STREAM("Trying to start controller " << (*it));
	}
	for(std::vector<std::string>::const_iterator it = stop_controllers.begin();
			it != stop_controllers.end();
			it++)
	{
		ROS_INFO_STREAM("Trying to stop controller " << (*it));
	}
	req.strictness =  pr2_mechanism_msgs::SwitchController::Request::BEST_EFFORT;
	if(!switch_controllers_service.call(req,res))
	{
		ROS_INFO("Call to switch controllers failed entirely");
	}
	if(res.ok != true)
	{
		ROS_INFO("Call to switch controllers reports not ok");
	}
}


/***********************************************************************************************************************
Main loop for testing
***********************************************************************************************************************/
int main(int argc, char **argv)
{
	// Init the ROS node
	ros::init(argc, argv, "pr2_cart_manager");

	ROS_INFO("### Starting pr2_cart_manager ###");

	PR2CartManager manager;

	manager.printState();

	manager.initGains();

//	ROS_INFO("PRESS [ENTER] TO CLOSE GRIPPERS AND START CONTROLLER");
//	getchar();
//	manager.on();
//
//	ROS_INFO("PRESS [ENTER] TO STOP AND OPEN GRIPPERS");
//	getchar();
//	manager.off();


	ROS_INFO("### Stopping pr2_cart_manager ###");




	ros::shutdown();

	return 0;
}


