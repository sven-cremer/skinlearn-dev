/*
 * SystemModel.h
 *
 *  Created on: Sep 7, 2013
 *      Author: Isura
 */

#ifndef SYSTEMMODEL_H_
#define SYSTEMMODEL_H_

#include <Eigen/Geometry>
#include <kdl/jntarray.hpp>

class SystemModel
{

  // Declare the number of joints.
  enum
  {
    Joints = 7
  };

  // Define the joint/cart vector types accordingly (using a fixed
  // size to avoid dynamic allocations and make the code realtime safe).
  Eigen::Matrix<double, Joints, Joints>  Mm;
  Eigen::Matrix<double, Joints, Joints>  Dm;
  Eigen::Matrix<double, Joints, Joints>  Km;
  Eigen::Matrix<double, Joints, 1>		 q_m;
  Eigen::Matrix<double, Joints, 1>       qd_m;
  Eigen::Matrix<double, Joints, 1>  	 qdd_m;
  Eigen::Matrix<double, Joints, 1>  	 t_h;
  Eigen::Matrix<double, Joints, Joints>  MmInv;

  double delT;

  // Ensure 128-bit alignment for Eigen
  // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 public:

	SystemModel();
	virtual ~SystemModel();

  /*
   * Updates the model
   */
  void init( double m, double d, double k );
  void update( KDL::JntArray & tau_ );
  void getStates( KDL::JntArray & q_m_, KDL::JntArray & qd_m_, KDL::JntArray & qdd_m_ );
  void debug();

};

#endif /* SYSTEMMODEL_H_ */
