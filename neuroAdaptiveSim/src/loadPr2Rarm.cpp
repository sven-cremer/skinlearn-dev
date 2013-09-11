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

  Model* RBDL_model = NULL;

  unsigned int RBDL_base_id					;
  unsigned int RBDL_r_shoulder_pan_link_id   ;
  unsigned int RBDL_r_shoulder_lift_link_id  ;
  unsigned int RBDL_r_upper_arm_roll_link_id ;
  unsigned int RBDL_r_upper_arm_link_id      ;
  unsigned int RBDL_r_elbow_flex_link_id     ;
  unsigned int RBDL_r_forearm_roll_link_id   ;
  unsigned int RBDL_r_forearm_link_id        ;
  unsigned int RBDL_r_wrist_flex_link_id     ;
  unsigned int RBDL_r_wrist_roll_link_id     ;

  Body RBDL_base             	 ;
  Body RBDL_r_shoulder_pan_link   ;
  Body RBDL_r_shoulder_lift_link  ;
  Body RBDL_r_upper_arm_roll_link ;
  Body RBDL_r_upper_arm_link      ;
  Body RBDL_r_elbow_flex_link     ;
  Body RBDL_r_forearm_roll_link   ;
  Body RBDL_r_forearm_link        ;
  Body RBDL_r_wrist_flex_link     ;
  Body RBDL_r_wrist_roll_link     ;

  Joint RBDL_r_shoulder_pan_joint   ;
  Joint RBDL_r_shoulder_lift_joint  ;
  Joint RBDL_r_upper_arm_roll_joint ;

  Joint RBDL_r_elbow_flex_joint     ;
  Joint RBDL_r_forearm_roll_joint   ;

  Joint RBDL_r_wrist_flex_joint     ;
  Joint RBDL_r_wrist_roll_joint     ;

  RBDL_model = new Model();

  RBDL_model->gravity = Vector3d (0., -9.81, 0.);

  RBDL_base             	   = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  RBDL_r_shoulder_pan_link     = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  RBDL_r_shoulder_lift_link    = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  RBDL_r_upper_arm_roll_link   = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  RBDL_r_upper_arm_link        = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  RBDL_r_elbow_flex_link       = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  RBDL_r_forearm_roll_link     = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  RBDL_r_forearm_link          = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  RBDL_r_wrist_flex_link       = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));
  RBDL_r_wrist_roll_link       = Body (1., Vector3d (0.5, 0., 0.0), Vector3d (1., 1., 1.));

  RBDL_r_shoulder_pan_joint   = Joint( JointTypeRevolute, Vector3d (0., 0., 1.) );
  RBDL_r_shoulder_lift_joint  = Joint( JointTypeRevolute, Vector3d (0., 1., 0.) );
  RBDL_r_upper_arm_roll_joint = Joint( JointTypeRevolute, Vector3d (1., 0., 0.) );
  RBDL_r_elbow_flex_joint     = Joint( JointTypeRevolute, Vector3d (0., 1., 0.) );
  RBDL_r_forearm_roll_joint   = Joint( JointTypeRevolute, Vector3d (1., 0., 0.) );
  RBDL_r_wrist_flex_joint     = Joint( JointTypeRevolute, Vector3d (0., 1., 0.) );
  RBDL_r_wrist_roll_joint     = Joint( JointTypeRevolute, Vector3d (1., 0., 0.) );

  RBDL_base_id					 = RBDL_model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), RBDL_r_shoulder_pan_joint   , RBDL_base				  );
  RBDL_r_shoulder_pan_link_id    = RBDL_model->AddBody(0, Xtrans(Vector3d(    0., -0.188, 0. )), RBDL_r_shoulder_lift_joint  , RBDL_r_shoulder_pan_link   );
  RBDL_r_shoulder_lift_link_id   = RBDL_model->AddBody(0, Xtrans(Vector3d(   0.1,     0., 0. )), RBDL_r_upper_arm_roll_joint , RBDL_r_shoulder_lift_link  );
  RBDL_r_upper_arm_roll_link_id  = RBDL_model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), JointTypeFixed              , RBDL_r_upper_arm_roll_link );
  RBDL_r_upper_arm_link_id       = RBDL_model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), JointTypeFixed              , RBDL_r_upper_arm_link      );
  RBDL_r_elbow_flex_link_id      = RBDL_model->AddBody(0, Xtrans(Vector3d(   0.4,     0., 0. )), RBDL_r_elbow_flex_joint     , RBDL_r_elbow_flex_link     );
  RBDL_r_forearm_roll_link_id    = RBDL_model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), RBDL_r_forearm_roll_joint   , RBDL_r_forearm_roll_link   );
  RBDL_r_forearm_link_id         = RBDL_model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), JointTypeFixed              , RBDL_r_forearm_link        );
  RBDL_r_wrist_flex_link_id      = RBDL_model->AddBody(0, Xtrans(Vector3d( 0.321,     0., 0. )), RBDL_r_wrist_flex_joint     , RBDL_r_wrist_flex_link     );
  RBDL_r_wrist_roll_link_id      = RBDL_model->AddBody(0, Xtrans(Vector3d(    0.,     0., 0. )), RBDL_r_wrist_roll_joint     , RBDL_r_wrist_roll_link     );


  VectorNd RBDL_Q     = VectorNd::Zero (RBDL_model->dof_count);
  VectorNd RBDL_QDot  = VectorNd::Zero (RBDL_model->dof_count);
  VectorNd RBDL_Tau   = VectorNd::Ones (RBDL_model->dof_count);
  VectorNd RBDL_QDDot = VectorNd::Zero (RBDL_model->dof_count);

  ForwardDynamics (*RBDL_model, RBDL_Q, RBDL_QDot, RBDL_Tau, RBDL_QDDot);

  std::cout << RBDL_QDDot.transpose() << " | DOF : " << RBDL_model->dof_count << std::endl;

  delete RBDL_model;

  return 0;
}



