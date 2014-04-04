/*
 * SystemModel_test.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: Isura
 */

#define EIGEN_RUNTIME_NO_MALLOC // Define this symbol to enable runtime tests for allocations

#include <SystemModel.h>

#include "csl/outer_loop.h"

int main()
{
//	KDL::JntArray  tau_;
//	tau_.resize( 7 );
//
//	tau_(0) = 0.5;
//	tau_(1) = 0.5;
//	tau_(2) = 0.5;
//	tau_(3) = 0.5;
//	tau_(4) = 0.5;
//	tau_(5) = 0.5;
//	tau_(6) = 0.5;
//
//	SystemModel testClass;

//	testClass.update( tau_ );
//	testClass.debug();
//
//	testClass.update( tau_ );
//	testClass.debug();
//
//	testClass.update( tau_ );
//	testClass.debug();
//
//	testClass.update( tau_ );
//	testClass.debug();

  Eigen::MatrixXd q;
  Eigen::MatrixXd qd;
  Eigen::MatrixXd qdd;

  Eigen::MatrixXd q_m;
  Eigen::MatrixXd qd_m;
  Eigen::MatrixXd qdd_m;
  Eigen::MatrixXd t_r;
  Eigen::MatrixXd task_ref;
  Eigen::MatrixXd tau;

  double num_Inputs  = 35 ;
  double num_Outputs = 7  ;
  double num_Hidden  = 10 ;
  double num_Error   = 7  ;
  double num_Joints  = 1  ;

  q       .resize( num_Joints, 1 ) ;
  qd      .resize( num_Joints, 1 ) ;
  qdd     .resize( num_Joints, 1 ) ;
  q_m     .resize( num_Joints, 1 ) ;
  qd_m    .resize( num_Joints, 1 ) ;
  qdd_m   .resize( num_Joints, 1 ) ;
  t_r     .resize( num_Joints, 1 ) ;
  task_ref.resize( num_Joints, 1 ) ;
  tau     .resize( num_Joints, 1 ) ;

  q         = Eigen::MatrixXd::Zero( num_Joints, 1 );
  qd        = Eigen::MatrixXd::Zero( num_Joints, 1 );
  qdd       = Eigen::MatrixXd::Zero( num_Joints, 1 );

  q_m       = Eigen::MatrixXd::Zero( num_Joints, 1 );
  qd_m      = Eigen::MatrixXd::Zero( num_Joints, 1 );
  qdd_m     = Eigen::MatrixXd::Zero( num_Joints, 1 );

  t_r       = Eigen::MatrixXd::Ones( num_Joints, 1 );
  task_ref  = Eigen::MatrixXd::Ones( num_Joints, 1 );
  tau       = Eigen::MatrixXd::Ones( num_Joints, 1 );

  csl::outer_loop::RlsModel outerLoopFIRmodel;
  outerLoopFIRmodel.updateDelT( 0.001 );

//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  t_r = t_r*7;
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  t_r = t_r*7;
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );
//  outerLoopFIRmodel.Update( qd_m  , qd    , q_m   , q     , qdd_m , t_r, task_ref );

	return 0;
}
