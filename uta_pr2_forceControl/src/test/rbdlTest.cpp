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
 * rbdlTest.cpp
 *  Created on: Mar 14, 2014
 */

#include <ros/ros.h>

#include "rbdl.h"
#include "rbdl_utils.h"
#include "rbdl_urdfreader.h"

#include <urdf_model/model.h>
#include <urdf_parser/urdf_parser.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main(int argc, char** argv)
{

  ros::init(argc, argv, "rbdlTest");

  ros::NodeHandle* rosnode = new ros::NodeHandle();

  std::string s_urdfString;
  rosnode->getParam( "/robot_description", s_urdfString );
//  urdf::Model urdfModel;
//  urdfModel.initString( s_urdfString );

  RigidBodyDynamics::Model* Rmodel;
  RigidBodyDynamics::Model* Lmodel;

  Rmodel = new Model();

  Rmodel->gravity = Vector3d (0., -9.81, 0.);

  bool verbose = true;
  if (!RigidBodyDynamics::Addons::read_urdf_model(s_urdfString.c_str(), Rmodel, verbose))
  {
    std::cerr << "Loading of urdf model failed!" << std::endl;
    return -1;
  }

  //RigidBodyDynamics::InverseDynamics ( model, Q, QDot, QDDot,  Tau); //, &f_ext);

  return 0;
}
