/*
 * position_rarm.cpp
 *
 *  Created on: Feb 1, 2014
 *      Author: isura
 */

#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>

trajectory_msgs::JointTrajectory armPointTrajectory( double pos_0,
                                                     double pos_1,
                                                     double pos_2,
                                                     double pos_3,
                                                     double pos_4,
                                                     double pos_5,
                                                     double pos_6 )
{
  //our goal variable
  trajectory_msgs::JointTrajectory goal;

  // First, the joint names, which apply to all waypoints
  goal.joint_names.push_back("r_shoulder_pan_joint");
  goal.joint_names.push_back("r_shoulder_lift_joint");
  goal.joint_names.push_back("r_upper_arm_roll_joint");
  goal.joint_names.push_back("r_elbow_flex_joint");
  goal.joint_names.push_back("r_forearm_roll_joint");
  goal.joint_names.push_back("r_wrist_flex_joint");
  goal.joint_names.push_back("r_wrist_roll_joint");

  // We will have two waypoints in this goal trajectory
  goal.points.resize(1);

  // First trajectory point
  // Positions
  int ind = 0;
  goal.points[ind].positions.resize(7);
  goal.points[ind].positions[0] = pos_0 ;
  goal.points[ind].positions[1] = pos_1 ;
  goal.points[ind].positions[2] = pos_2 ;
  goal.points[ind].positions[3] = pos_3 ;
  goal.points[ind].positions[4] = pos_4 ;
  goal.points[ind].positions[5] = pos_5 ;
  goal.points[ind].positions[6] = pos_6 ;
  // Velocities
  goal.points[ind].velocities.resize(7);
  for (size_t j = 0; j < 7; ++j)
  {
    goal.points[ind].velocities[j] = 0.0;
  }

  // To be reached 2 second after starting along the trajectory
  goal.points[ind].time_from_start = ros::Duration(2);

  //we are done; return the goal
  return goal;
}

int main(int argc, char** argv)
{

  ros::init(argc, argv, "my_tf_listener");

  ros::NodeHandle node;

  ros::Publisher joint_pub = node.advertise<trajectory_msgs::JointTrajectory>("/r_arm_controller/command", 1);

  ros::Rate rate(30.0);

  float pi = 3.14159265359;

  while (node.ok())
  {

    joint_pub.publish( armPointTrajectory( -0.48577   ,
                                           -0.0190721 ,
                                           -1.51115   ,
                                           -1.70928   ,
                                            1.54561   ,
                                            0.046854  ,
                                           -0.0436174   ) );

    rate.sleep();
  }

  return 0;

}
;



