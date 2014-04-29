/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, UT Arlington
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of UT Arlington nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Isura Ranatunga
 *
 * refTrajectoryClient.cpp
 *  Created on: Apr 1, 2014
 */

#include <unistd.h>
#include <ros/ros.h>
#include <neuroadaptive_msgs/setCartPose.h>
#include <neuroadaptive_msgs/saveControllerData.h>
#include <std_srvs/Empty.h>
#include <sound_play/sound_play.h>

using namespace std;

class referenceTrajectoryClient
{
  ros::NodeHandle  node;
  ros::ServiceClient save_client;
  ros::ServiceClient setRefTraj_client;

  sound_play::SoundClient sc;

  ros::Time start_time;        // Time of the first servo cycle

  // Desired cartesian pose
  double cartDesX     ;
  double cartDesY     ;
  double cartDesZ     ;
  double cartDesRoll  ;
  double cartDesPitch ;
  double cartDesYaw   ;

  // Initial cartesian pose
  double cartIniX ;
  double cartIniY ;
  double cartIniZ ;

  // Flags
  bool redFlag     ;
  bool blueFlag    ;
  bool greenFlag   ;
  bool captureFlag ;
  bool runFlag     ;

  // No of switch
  // How many times switched positions
  uint switchNo    ;

public:

  referenceTrajectoryClient()
  {
    save_client     = node.serviceClient<neuroadaptive_msgs::saveControllerData>("pr2_cartneuroController/save");
    setRefTraj_client= node.serviceClient<neuroadaptive_msgs::setCartPose>("pr2_cartneuroController/setRefTraj");
    start_time = ros::Time::now();

    std::string para_cartDesX     = "/cartDesX";
    std::string para_cartDesY     = "/cartDesY";
    std::string para_cartDesZ     = "/cartDesZ";
    std::string para_cartDesRoll  = "/cartDesRoll";
    std::string para_cartDesPitch = "/cartDesPitch";
    std::string para_cartDesYaw   = "/cartDesYaw";


    std::string para_cartIniX     = "/cartIniX";
    std::string para_cartIniY     = "/cartIniY";
    std::string para_cartIniZ     = "/cartIniZ";

    if (!node.getParam( para_cartDesX     , cartDesX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesX    .c_str()) ; }
    if (!node.getParam( para_cartDesY     , cartDesY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesY    .c_str()) ; }
    if (!node.getParam( para_cartDesZ     , cartDesZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesZ    .c_str()) ; }
    if (!node.getParam( para_cartDesRoll  , cartDesRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesRoll .c_str()) ; }
    if (!node.getParam( para_cartDesPitch , cartDesPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesPitch.c_str()) ; }
    if (!node.getParam( para_cartDesYaw   , cartDesYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesYaw  .c_str()) ; }

    if (!node.getParam( para_cartIniX, cartIniX )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniX.c_str()) ; }
    if (!node.getParam( para_cartIniY, cartIniY )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniY.c_str()) ; }
    if (!node.getParam( para_cartIniZ, cartIniZ )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniZ.c_str()) ; }

    // Flags
    redFlag     = false ;
    blueFlag    = false ;
    greenFlag   = false ;
    captureFlag = false ;
    runFlag     = true  ;

    switchNo    = 0     ;

  }

  ~referenceTrajectoryClient() { }

  void sleepok(int t, ros::NodeHandle &nh)
  {
    if (nh.ok())
    sleep(t);
  }

  void go()
  {
    ROS_INFO_STREAM("# Starting Experiment #");
    start_time = ros::Time::now();

    while( ros::ok() && runFlag )
    {
      neuroadaptive_msgs::setCartPose setCartPoseAction;

      if( (ros::Time::now() - start_time).toSec() > 4 && !captureFlag )
      {
    	neuroadaptive_msgs::saveControllerData saveSrv;
    	saveSrv.request.fileName = "~/Dropbox/PhD/UTARI/PR2/PR2_TRO/PR2/ProtobufData/test1.txt";
        save_client.call(saveSrv);
        captureFlag = true ;
      }

      if( (ros::Time::now() - start_time).toSec() > 5 && !redFlag )
      {
    	ROS_INFO_STREAM("# BLUE # | Time: " << (ros::Time::now() - start_time).toSec() );
        sc.say("Blue!");
        sleepok(1, node);
        setCartPoseAction.request.msg.position.x = cartDesX ;
        setCartPoseAction.request.msg.position.y = cartDesY ;
        setCartPoseAction.request.msg.position.z = cartDesZ ;
        redFlag = true ;
        setRefTraj_client.call(setCartPoseAction);
        switchNo++;
      }

      if( (ros::Time::now() - start_time).toSec() > 10 && !greenFlag )
      {
    	ROS_INFO_STREAM("# RED  # | Time: " << (ros::Time::now() - start_time).toSec() );
        sc.say("Red!");
        sleepok(1, node);
        setCartPoseAction.request.msg.position.x = cartIniX ;
        setCartPoseAction.request.msg.position.y = cartIniY ;
        setCartPoseAction.request.msg.position.z = cartIniZ ;
        setRefTraj_client.call(setCartPoseAction);
        greenFlag = true ;

        start_time = ros::Time::now();
        redFlag   = false ;
        blueFlag  = false ;
        greenFlag = false ;
        switchNo++;
      }

      sleep(0.5);

      if( switchNo > 4 )
	  {
    	  captureFlag = false ;
    	  switchNo    = 0     ;
    	  ROS_INFO_STREAM("# Experiment DONE #");
    	  ROS_INFO_STREAM("0 - no");
    	  ROS_INFO_STREAM("1 - yes");
    	  ROS_INFO_STREAM("Run again? :");

    	  bool runAgain;
    	  std::cin >> runAgain;

    	  if( runAgain )
    	  {
			  start_time = ros::Time::now();

			  neuroadaptive_msgs::saveControllerData saveSrv;
    		  saveSrv.request.fileName = "~/Dropbox/PhD/UTARI/PR2/PR2_TRO/PR2/ProtobufData/test1.txt";
			  save_client.call(saveSrv);

			  captureFlag = true  ;
			  redFlag     = false ;
			  blueFlag    = false ;
			  greenFlag   = false ;
    	  }else
    	  {
    		  runFlag = false ;
    	  }

	  }

//      if( (int) ceil( (ros::Time::now() - start_time).toSec() ) % 6 == 0 )
//      {
//        setCartPoseAction.request.msg.position.x = cartDesX ;
//        setCartPoseAction.request.msg.position.y = cartDesY ;
//        setCartPoseAction.request.msg.position.z = cartDesZ ;
//        ROS_ERROR_STREAM("# BLUE #");
//      }

    }
  }

};

int
main( int argc, char** argv )
{
    // Initialize ROS
    ros::init (argc, argv, "setCartesianReference_client");
    ros::NodeHandle nh;

    referenceTrajectoryClient cartRefObj;

    cartRefObj.go();
}

