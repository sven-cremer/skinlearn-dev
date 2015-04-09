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
#include "rbdl_urdfreader.cc"

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

  std::cout << s_urdfString;

  std::string root_name = "r_shoulder_pan_link";
  std::string tip_name = "r_shoulder_pan_link"; // "r_gripper_tool_frame";

  if (!RigidBodyDynamics::Addons::read_urdf_model(s_urdfString.c_str(), Rmodel, verbose, root_name, tip_name))
  {
    std::cerr << "Loading of urdf model failed!" << std::endl;
    return -1;
  }

  std::cout << std::endl << "DOF: " <<  Rmodel->dof_count << endl;

//  for( int i = 0; i < Rmodel->dof_count; i++ )
//  {
//    std::cout << std::endl << "Body " << i << " : " << Rmodel->mBodyNameMap;
//  }

  std::map<std::string, unsigned int> bodyMap = Rmodel->mBodyNameMap;

  typedef std::map< string, unsigned int >::const_iterator MapIterator;
  for (MapIterator iter = bodyMap.begin(); iter != bodyMap.end(); iter++)
  {
      cout << "Key: " << iter->first << " | Values: " << iter->second << endl;
  }


  RigidBodyDynamics::Model model = *Rmodel;

  cout << "Degree of freedom overview:" << endl;
  cout << RigidBodyDynamics::Utils::GetModelDOFOverview(model);

  MatrixNd H = MatrixNd::Zero (model.dof_count, model.dof_count);
  VectorNd Q = VectorNd::Zero (model.dof_count);
  VectorNd QDot = VectorNd::Zero (model.dof_count);
  VectorNd Tau = VectorNd::Ones (model.dof_count);
  VectorNd QDDot = VectorNd::Zero (model.dof_count);

  std::cout << endl << "BEFORE: " << endl << H <<endl << endl << Q.transpose() << endl;

  // Computes the joint space inertia matrix by using the Composite Rigid Body Algorithm
  RigidBodyDynamics::CompositeRigidBodyAlgorithm( model, Q, H, false );

  std::cout << endl << "AFTER: " << endl << H <<endl << endl << Q.transpose() << endl;

  // Computes forward dynamics with the Articulated Body Algorithm
  RigidBodyDynamics::ForwardDynamics ( model, Q, QDot, Tau, QDDot );

  std::cout << "Q: "<< QDDot.transpose() << std::endl << std::endl;

  RigidBodyDynamics::InverseDynamics ( model, Q, QDot, QDDot, Tau );

  std::cout << "Tau: "<< Tau.transpose() << std::endl << std::endl;

  return 0;
}
