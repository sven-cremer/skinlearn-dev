/*
 * ft_simulation.cpp
 *
 *  Created on: Apr 28, 2014
 *      Author: sven
 */

#include "ros/ros.h"
#include "gazebo_msgs/ApplyBodyWrench.h"
#include <cstdlib>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ft_simulation");


  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

  gazebo_msgs::ApplyBodyWrench f1;
  gazebo_msgs::ApplyBodyWrench f2;


  f1.request.body_name =	 	"pr2::r_gripper_l_finger_tip_link";
//  f1.request.reference_frame =	"pr2::r_gripper_l_finger_tip_link";
//  f1.request.reference_frame =	"pr2::torso_lift_link";


  f1.request.reference_point.x = 0;
  f1.request.reference_point.y = 0;
  f1.request.reference_point.z = 0;

  f1.request.wrench.force.x =	 25.0;
  f1.request.wrench.force.y =	 0.0;
  f1.request.wrench.force.z =	 0.0;
  f1.request.wrench.torque.x =	 0.0;
  f1.request.wrench.torque.y =	 0.0;
  f1.request.wrench.torque.z =	 0.0;



  f2.request.body_name =	 	"pr2::l_gripper_l_finger_tip_link";
//  f2.request.reference_frame =	"pr2::l_gripper_l_finger_tip_link";
//  f2.request.reference_frame =	"pr2::torso_lift_link";

  f2.request.reference_point.x = 0;
  f2.request.reference_point.y = 0;
  f2.request.reference_point.z = 0;

  f2.request.wrench.force.x =	 -25.0;
  f2.request.wrench.force.y =	 0.0;
  f2.request.wrench.force.z =	 0.0;
  f2.request.wrench.torque.x =	 0.0;
  f2.request.wrench.torque.y =	 0.0;
  f2.request.wrench.torque.z =	 0.0;



  ros::Time::waitForValid();
  ros::Time begin = ros::Time::now();
  ros::Duration delay(1,0);				// 1 second delay


  f1.request.start_time = begin + delay;
  f1.request.duration	= ros::Duration(5);

  f2.request.start_time = begin + delay;
  f2.request.duration	= ros::Duration(5);


  if (n.ok())
  {
    ROS_INFO("Applying forces ...");

    bool a = client.call(f1);
    bool b = client.call(f2);

    ROS_INFO("Response F1: %d",f1.response.success);
    ROS_INFO("Response F2: %d",f2.response.success);


//    if(client.call(f1))
//    {
//    	ROS_INFO("Response: %d",f1.response.success);
//    }
//    {
//    	ROS_INFO("Failed to call client!");
//    }
//
//    if(client.call(f2))
//    {
//    	ROS_INFO("Response: %d",f2.response.success);
//    }
//    {
//    	ROS_INFO("Failed to call client!");
//    }

  }
  else
  {
    ROS_ERROR("Shutdown already in progress. Cannot apply forces ...");
  }


}

//rosservice call /gazebo/apply_body_wrench '{body_name: "pr2::r_gripper_l_finger_tip_link",reference_frame: "pr2::r_gripper_l_finger_tip_link", wrench: { force: { x: 0.0, y: 10.0, z: 0 } }, start_time: 10000000000, duration: 5000000000 }'
//rosservice call /gazebo/apply_body_wrench '{body_name: "pr2::l_gripper_l_finger_tip_link",reference_frame: "pr2::l_gripper_l_finger_tip_link", wrench: { force: { x: 0.0, y: 10.0, z: 0 } }, start_time: 10000000000, duration: 5000000000 }'
