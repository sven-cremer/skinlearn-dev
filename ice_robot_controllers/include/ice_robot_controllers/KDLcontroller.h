/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
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

// Original Author: Stuart Glaser
// Original file:   JT Cartesian controller

/*
 * KDLcontroller.h
 *
 *  Created on: Nov 6, 2015
 *      Author: Sven Cremer
 */

#ifndef KDLCONTROLLER_H_
#define KDLCONTROLLER_H_


#include <Eigen/Core>

#include <eigen_conversions/eigen_kdl.h>
#include <eigen_conversions/eigen_msg.h>
#include <kdl/chainfksolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/frames.hpp>


namespace pr2_controller_ns{


template <int Joints>
struct Kin
{
  typedef Eigen::Matrix<double, Joints, 1> JointVec;
  typedef Eigen::Matrix<double, 6, Joints> Jacobian;

  Kin(const KDL::Chain &kdl_chain) :
    fk_solver_(kdl_chain), jac_solver_(kdl_chain),
    kdl_q(Joints), kdl_J(Joints)
  {
  }
  ~Kin()
  {
  }

  void fk(const JointVec &q, Eigen::Affine3d &x)
  {
    kdl_q.data = q;
    KDL::Frame kdl_x;
    fk_solver_.JntToCart(kdl_q, kdl_x);
    tf::transformKDLToEigen(kdl_x, x);
  }
  void jac(const JointVec &q, Jacobian &J)
  {
    kdl_q.data = q;
    jac_solver_.JntToJac(kdl_q, kdl_J);
    J = kdl_J.data;
  }

  KDL::ChainFkSolverPos_recursive fk_solver_;
  KDL::ChainJntToJacSolver jac_solver_;
  KDL::JntArray kdl_q;
  KDL::Jacobian kdl_J;
};

static void computePoseError(const Eigen::Affine3d &xact, const Eigen::Affine3d &xdes, Eigen::Matrix<double,6,1> &err)
{
  err.head<3>() = xact.translation() - xdes.translation();
  err.tail<3>()   = 0.5 * (xdes.linear().col(0).cross(xact.linear().col(0)) +
                          xdes.linear().col(1).cross(xact.linear().col(1)) +
                          xdes.linear().col(2).cross(xact.linear().col(2)));
}



inline void computeNullspace(Eigen::Matrix<double, 6, 7> J_,
		                 Eigen::Matrix<double, 7, 7> N_)

{
	// ======== J psuedo-inverse and Nullspace computation
	const int Joints = 7;
	double k_posture = 25.0;
	double jacobian_inverse_damping = 0.01;

	// Computes pseudo-inverse of J
	Eigen::Matrix<double,6,6> I6; I6.setIdentity();
	Eigen::Matrix<double,6,6> JJt_damped = J_ * J_.transpose() + jacobian_inverse_damping * I6;
	Eigen::Matrix<double,6,6> JJt_inv_damped = JJt_damped.inverse();
	Eigen::Matrix<double,Joints,6> J_pinv = J_.transpose() * JJt_inv_damped;

	// Computes the nullspace of J
	Eigen::Matrix<double,Joints,Joints> I;
	I.setIdentity();
	N_ = I - J_pinv * J_;

}


} // end namespace


#endif /* KDLCONTROLLER_H_ */
