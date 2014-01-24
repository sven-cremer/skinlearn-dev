/*
 * pr2_joint_listener.cpp
 *
 *  Created on: Jan 24, 2014
 *      Author: isura
 */

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

void jointCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
  std::vector<int> r_arm_jointID;
  r_arm_jointID.reserve(7);

  char r_arm_jointName[7][50];

  int j = 0;

  for ( int i = 0; i < msg->name.size(); i++ )
  {
    if( !strcmp( msg->name[i].c_str(), "r_shoulder_pan_joint"   ) ) { r_arm_jointID.push_back(i); strcpy(r_arm_jointName[j], msg->name[i].c_str()); ROS_INFO_STREAM("Name: " << msg->name[i] << "  " << i ); j++;}
    if( !strcmp( msg->name[i].c_str(), "r_shoulder_lift_joint"  ) ) { r_arm_jointID.push_back(i); strcpy(r_arm_jointName[j], msg->name[i].c_str()); ROS_INFO_STREAM("Name: " << msg->name[i] << "  " << i ); j++;}
    if( !strcmp( msg->name[i].c_str(), "r_upper_arm_roll_joint" ) ) { r_arm_jointID.push_back(i); strcpy(r_arm_jointName[j], msg->name[i].c_str()); ROS_INFO_STREAM("Name: " << msg->name[i] << "  " << i ); j++;}
    if( !strcmp( msg->name[i].c_str(), "r_elbow_flex_joint"     ) ) { r_arm_jointID.push_back(i); strcpy(r_arm_jointName[j], msg->name[i].c_str()); ROS_INFO_STREAM("Name: " << msg->name[i] << "  " << i ); j++;}
    if( !strcmp( msg->name[i].c_str(), "r_forearm_roll_joint"   ) ) { r_arm_jointID.push_back(i); strcpy(r_arm_jointName[j], msg->name[i].c_str()); ROS_INFO_STREAM("Name: " << msg->name[i] << "  " << i ); j++;}
    if( !strcmp( msg->name[i].c_str(), "r_wrist_flex_joint"     ) ) { r_arm_jointID.push_back(i); strcpy(r_arm_jointName[j], msg->name[i].c_str()); ROS_INFO_STREAM("Name: " << msg->name[i] << "  " << i ); j++;}
    if( !strcmp( msg->name[i].c_str(), "r_wrist_roll_joint"     ) ) { r_arm_jointID.push_back(i); strcpy(r_arm_jointName[j], msg->name[i].c_str()); ROS_INFO_STREAM("Name: " << msg->name[i] << "  " << i ); j++;}

  }

  ROS_INFO_STREAM("Name                     Pos");
  ROS_INFO_STREAM( msg->position[ r_arm_jointID.at(1) ] << " : "  << r_arm_jointID.at(1) << " : "  << r_arm_jointName[1] );
  ROS_INFO_STREAM( msg->position[ r_arm_jointID.at(2) ] << " : "  << r_arm_jointID.at(2) << " : "  << r_arm_jointName[2] );
  ROS_INFO_STREAM( msg->position[ r_arm_jointID.at(0) ] << " : "  << r_arm_jointID.at(0) << " : "  << r_arm_jointName[0] );
  ROS_INFO_STREAM( msg->position[ r_arm_jointID.at(4) ] << " : "  << r_arm_jointID.at(4) << " : "  << r_arm_jointName[4] );
  ROS_INFO_STREAM( msg->position[ r_arm_jointID.at(3) ] << " : "  << r_arm_jointID.at(3) << " : "  << r_arm_jointName[3] );
  ROS_INFO_STREAM( msg->position[ r_arm_jointID.at(5) ] << " : "  << r_arm_jointID.at(5) << " : "  << r_arm_jointName[5] );
  ROS_INFO_STREAM( msg->position[ r_arm_jointID.at(6) ] << " : "  << r_arm_jointID.at(6) << " : "  << r_arm_jointName[6] );
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "joint_listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/joint_states", 1, jointCallback);

  ros::spin();

  return 0;
}
