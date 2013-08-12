/*
 * kinematichelpers.h
 *
 *  Created on: Aug 12, 2013
 *      Author: isura
 */

#ifndef KINEMATICHELPERS_H_
#define KINEMATICHELPERS_H_

#include <boost/scoped_ptr.hpp>
#include <kdl/chain.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>
#include <kdl/jacobian.hpp>
#include <kdl/jntarray.hpp>

#include <Eigen/Geometry>

namespace pr2_mechanism_model{

class KinematicHelpers
{

  // Declare the number of joints.
  enum
  {
    Joints = 7
  };

  // Define the joint/cart vector types accordingly (using a fixed
  // size to avoid dynamic allocations and make the code realtime safe).
  typedef Eigen::Matrix<double, Joints, 1>  JointVector;
  typedef Eigen::Affine3d                   CartPose;
  typedef Eigen::Matrix<double, 3, 1>       Cart3Vector;
  typedef Eigen::Matrix<double, 6, 1>       Cart6Vector;
  typedef Eigen::Matrix<double, 6, Joints>  JacobianMatrix;

  // Ensure 128-bit alignment for Eigen
  // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:

  void KDLtoEigen( KDL::Frame     & xtmp_ , CartPose       & x0_                        );
  void EigentoKDL( JointVector    & q     , KDL::JntArray  & qtmp_                      );
  void KDLtoEigen( KDL::Jacobian  & Jtmp_ , JacobianMatrix & J                          );
  void computeCartError( CartPose & x     , CartPose       & xd    , Cart6Vector & xerr );

};
}

#endif /* KINEMATICHELPERS_H_ */
