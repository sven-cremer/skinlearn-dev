/*
 * loadPr2Rarm.cpp
 *
 *  Created on: Sep 11, 2013
 *      Author: isura
 */

#include <rbdl/rbdl.h>

using namespace RigidBodyDynamics;
using namespace RigidBodyDynamics::Math;

int main (int argc, char* argv[]) {
  rbdl_check_api_version (RBDL_API_VERSION);

  Model* model = NULL;

  unsigned int base_id					;
  unsigned int r_shoulder_pan_link_id   ;
  unsigned int r_shoulder_lift_link_id  ;
  unsigned int r_upper_arm_roll_link_id ;
  unsigned int r_upper_arm_link_id      ;
  unsigned int r_elbow_flex_link_id     ;
  unsigned int r_forearm_roll_link_id   ;
  unsigned int r_forearm_link_id        ;
  unsigned int r_wrist_flex_link_id     ;
  unsigned int r_wrist_roll_link_id     ;

  Body base             	 ;
  Body r_shoulder_pan_link   ;
  Body r_shoulder_lift_link  ;
  Body r_upper_arm_roll_link ;
  Body r_upper_arm_link      ;
  Body r_elbow_flex_link     ;
  Body r_forearm_roll_link   ;
  Body r_forearm_link        ;
  Body r_wrist_flex_link     ;
  Body r_wrist_roll_link     ;

  Joint r_shoulder_pan_joint   ;
  Joint r_shoulder_lift_joint  ;
  Joint r_upper_arm_roll_joint ;

  Joint r_elbow_flex_joint     ;
  Joint r_forearm_roll_joint   ;

  Joint r_wrist_flex_joint     ;
  Joint r_wrist_roll_joint     ;

  model = new Model();

  model->gravity = Vector3d (0., -9.81, 0.);

  base             	      = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  r_shoulder_pan_link     = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  r_shoulder_lift_link    = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  r_upper_arm_roll_link   = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  r_upper_arm_link        = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  r_elbow_flex_link       = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  r_forearm_roll_link     = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  r_forearm_link          = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  r_wrist_flex_link    	  = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  r_wrist_roll_link       = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));

  r_shoulder_pan_joint   = Joint( JointTypeRevolute, Vector3d (0., 0., 1.) );
  r_shoulder_lift_joint  = Joint( JointTypeRevolute, Vector3d (0., 1., 0.) );
  r_upper_arm_roll_joint = Joint( JointTypeRevolute, Vector3d (1., 0., 0.) );
  r_elbow_flex_joint     = Joint( JointTypeRevolute, Vector3d (0., 1., 0.) );
  r_forearm_roll_joint   = Joint( JointTypeRevolute, Vector3d (1., 0., 0.) );
  r_wrist_flex_joint     = Joint( JointTypeRevolute, Vector3d (0., 1., 0.) );
  r_wrist_roll_joint     = Joint( JointTypeRevolute, Vector3d (1., 0., 0.) );

  base_id					= model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), r_shoulder_pan_joint   , base				  );
  r_shoulder_pan_link_id    = model->AddBody(0, Xtrans(Vector3d(    0., -0.188, 0. )), r_shoulder_lift_joint  , r_shoulder_pan_link   );
  r_shoulder_lift_link_id   = model->AddBody(0, Xtrans(Vector3d(   0.1,     0., 0. )), r_upper_arm_roll_joint , r_shoulder_lift_link  );
  r_upper_arm_roll_link_id  = model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), JointTypeFixed         , r_upper_arm_roll_link );
  r_upper_arm_link_id       = model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), JointTypeFixed         , r_upper_arm_link      );
  r_elbow_flex_link_id      = model->AddBody(0, Xtrans(Vector3d(   0.4,     0., 0. )), r_elbow_flex_joint     , r_elbow_flex_link     );
  r_forearm_roll_link_id    = model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), r_forearm_roll_joint   , r_forearm_roll_link   );
  r_forearm_link_id         = model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), JointTypeFixed         , r_forearm_link        );
  r_wrist_flex_link_id     	= model->AddBody(0, Xtrans(Vector3d( 0.321,     0., 0. )), r_wrist_flex_joint     , r_wrist_flex_link     );
  r_wrist_roll_link_id     	= model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), r_wrist_roll_joint     , r_wrist_roll_link     );


  VectorNd Q     = VectorNd::Zero (model->dof_count);
  VectorNd QDot  = VectorNd::Zero (model->dof_count);
  VectorNd Tau   = VectorNd::Ones (model->dof_count);
  VectorNd QDDot = VectorNd::Zero (model->dof_count);

  ForwardDynamics (*model, Q, QDot, Tau, QDDot);

  std::cout << QDDot.transpose() << " | DOF : " << model->dof_count << std::endl;

  delete model;

  return 0;
}



