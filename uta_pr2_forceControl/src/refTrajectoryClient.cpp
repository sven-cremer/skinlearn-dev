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

#include <vector>

#include <Eigen/Core>
#include <Eigen/StdVector>
#include <Eigen/Geometry>

using namespace std;

class referenceTrajectoryClient
{
  ros::NodeHandle  node;
  ros::ServiceClient save_client;
  ros::ServiceClient setRefTraj_client;

  sound_play::SoundClient sc;

  ros::Time start_time;        // Time of the first servo cycle

  // Desired cartesian pose
  double cartDesX      ;
  double cartDesY      ;
  double cartDesZ      ;
  double cartDesRoll   ;
  double cartDesPitch  ;
  double cartDesYaw    ;

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

  // Initial cartesian pose
  double cartIniX ;
  double cartIniY ;
  double cartIniZ ;

  // Flags
  bool blueFlag    ;
  bool redFlag     ;
  bool captureFlag ;
  bool runFlag     ;
  bool runAgainFlag;

  vector<string> pointNames;
  std::vector< Eigen::Vector3d,Eigen::aligned_allocator<Eigen::Vector3d> > desPoses ;

  // No of switch
  // How many times switched positions
  uint switchNo    ;

  int posIndex;

public:

  referenceTrajectoryClient()
  {
    save_client     = node.serviceClient<neuroadaptive_msgs::saveControllerData>("pr2_cartneuroController/save");
    setRefTraj_client= node.serviceClient<neuroadaptive_msgs::setCartPose>("pr2_cartneuroController/setRefTraj");
    start_time = ros::Time::now();

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

    std::string para_cartIniX     = "/cartIniX";
    std::string para_cartIniY     = "/cartIniY";
    std::string para_cartIniZ     = "/cartIniZ";

    if (!node.getParam( para_cartDesX     , cartDesX     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesX    .c_str()) ; }
    if (!node.getParam( para_cartDesY     , cartDesY     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesY    .c_str()) ; }
    if (!node.getParam( para_cartDesZ     , cartDesZ     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesZ    .c_str()) ; }
    if (!node.getParam( para_cartDesRoll  , cartDesRoll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesRoll .c_str()) ; }
    if (!node.getParam( para_cartDesPitch , cartDesPitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesPitch.c_str()) ; }
    if (!node.getParam( para_cartDesYaw   , cartDesYaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDesYaw  .c_str()) ; }

    if (!node.getParam( para_cartDes2X     , cartDes2X     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2X    .c_str()) ; }
    if (!node.getParam( para_cartDes2Y     , cartDes2Y     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2Y    .c_str()) ; }
    if (!node.getParam( para_cartDes2Z     , cartDes2Z     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2Z    .c_str()) ; }
    if (!node.getParam( para_cartDes2Roll  , cartDes2Roll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2Roll .c_str()) ; }
    if (!node.getParam( para_cartDes2Pitch , cartDes2Pitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2Pitch.c_str()) ; }
    if (!node.getParam( para_cartDes2Yaw   , cartDes2Yaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes2Yaw  .c_str()) ; }

    if (!node.getParam( para_cartDes3X     , cartDes3X     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3X    .c_str()) ; }
    if (!node.getParam( para_cartDes3Y     , cartDes3Y     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3Y    .c_str()) ; }
    if (!node.getParam( para_cartDes3Z     , cartDes3Z     )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3Z    .c_str()) ; }
    if (!node.getParam( para_cartDes3Roll  , cartDes3Roll  )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3Roll .c_str()) ; }
    if (!node.getParam( para_cartDes3Pitch , cartDes3Pitch )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3Pitch.c_str()) ; }
    if (!node.getParam( para_cartDes3Yaw   , cartDes3Yaw   )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartDes3Yaw  .c_str()) ; }


    if (!node.getParam( para_cartIniX, cartIniX )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniX.c_str()) ; }
    if (!node.getParam( para_cartIniY, cartIniY )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniY.c_str()) ; }
    if (!node.getParam( para_cartIniZ, cartIniZ )){ ROS_ERROR("Value not loaded from parameter: %s !)", para_cartIniZ.c_str()) ; }

    // Flags
    blueFlag    = false ;
    redFlag     = false ;
    captureFlag = false ;
    runFlag     = true  ;
    runAgainFlag= false ;

    switchNo    = 0     ;

    pointNames.push_back("Green!");
    desPoses.push_back( Eigen::Vector3d(cartDesX,
    		                            cartDesY,
    		                            cartDesZ) );

    pointNames.push_back("Blue!");
    desPoses.push_back( Eigen::Vector3d(cartDes2X,
    		                            cartDes2Y,
    		                            cartDes2Z) );

    pointNames.push_back("Yellow!");
    desPoses.push_back( Eigen::Vector3d(cartDes3X,
    		                            cartDes3Y,
    		                            cartDes3Z) );

    pointNames.push_back("Red!");
    desPoses.push_back( Eigen::Vector3d(cartIniX,
    		                            cartIniY,
    		                            cartIniZ) );
    posIndex = 0;

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
    sc.say("Starting Experiment!");
    sleepok(1, node);

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

      if( (ros::Time::now() - start_time).toSec() > 5 )
      {
    	ROS_INFO_STREAM("# " << pointNames[posIndex] << " # | Time: " << (ros::Time::now() - start_time).toSec() );
        sc.say(pointNames[posIndex]);
        sleepok(1, node);
        setCartPoseAction.request.msg.position.x = desPoses[posIndex].x() ;
        setCartPoseAction.request.msg.position.y = desPoses[posIndex].y() ;
        setCartPoseAction.request.msg.position.z = desPoses[posIndex].z() ;

        setRefTraj_client.call(setCartPoseAction);
        start_time = ros::Time::now();
        switchNo++;
        posIndex++;

        if( posIndex >= pointNames.size() )
        {
        	posIndex = 0;
        }

      }

//      if( (ros::Time::now() - start_time).toSec() > 5 && !blueFlag )
//      {
//    	ROS_INFO_STREAM("# BLUE # | Time: " << (ros::Time::now() - start_time).toSec() );
//        sc.say("Red!");
//        sleepok(1, node);
//        setCartPoseAction.request.msg.position.x = cartDesX ;
//        setCartPoseAction.request.msg.position.y = cartDesY ;
//        setCartPoseAction.request.msg.position.z = cartDesZ ;
//        blueFlag = true ;
//        setRefTraj_client.call(setCartPoseAction);
//        switchNo++;
//      }
//
//      if( (ros::Time::now() - start_time).toSec() > 10 && !redFlag )
//      {
//    	ROS_INFO_STREAM("# RED  # | Time: " << (ros::Time::now() - start_time).toSec() );
//        sc.say("Blue!");
//        sleepok(1, node);
//        setCartPoseAction.request.msg.position.x = cartIniX ;
//        setCartPoseAction.request.msg.position.y = cartIniY ;
//        setCartPoseAction.request.msg.position.z = cartIniZ ;
//        setRefTraj_client.call(setCartPoseAction);
//        redFlag = true ;
//
//        start_time = ros::Time::now();
//        blueFlag   = false ;
//        redFlag = false ;
//        switchNo++;
//      }

      if( switchNo > 11 )
	  {
    	  sleep(2);
		  captureFlag = false ;
		  switchNo    = 0     ;
		  posIndex    = 0     ;
		  ROS_INFO_STREAM("# Experiment DONE #");
		  sc.say("Experiment complete!");
		  ROS_INFO_STREAM("0 - no");
		  ROS_INFO_STREAM("1 - yes");
		  ROS_INFO_STREAM("Run again? :");

//		  std::cin >> runAgainFlag;

		  if( runAgainFlag )
		  {
			  start_time = ros::Time::now();

			  neuroadaptive_msgs::saveControllerData saveSrv;
			  saveSrv.request.fileName = "~/Dropbox/PhD/UTARI/PR2/PR2_TRO/PR2/ProtobufData/test1.txt";
			  save_client.call(saveSrv);

			  captureFlag = true  ;
			  blueFlag    = false ;
			  redFlag     = false ;

			  ROS_INFO_STREAM("# Starting Experiment #");
			  sc.say("Starting Experiment!");
			  sleepok(1, node);

		  }else
		  {
			  runFlag = false ;
			  break;
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

