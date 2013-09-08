/*
 * SystemModel.h
 *
 *  Created on: Sep 7, 2013
 *      Author: Isura
 */

#ifndef SYSTEMMODEL_H_
#define SYSTEMMODEL_H_

#include <Eigen/Geometry>
//#include <kdl/jntarray.hpp>

class SystemModel
{

  /*
  // Declare the number of joints.
  enum
  {
    Joints = 7
  };

  // Define the joint/cart vector types accordingly (using a fixed
  // size to avoid dynamic allocations and make the code realtime safe).
  typedef Eigen::Matrix<double, Joints, Joints>  SystemMatrix;
  typedef Eigen::Matrix<double, Joints, 1>		 SystemVector;

  SystemMatrix  Mm;
  SystemMatrix  Dm;
  SystemMatrix  Km;
  SystemMatrix  MmInv;

  SystemVector	q_m;
  SystemVector  qd_m;
  SystemVector 	qdd_m;
  SystemVector 	t_h;

  double delT;

  // Ensure 128-bit alignment for Eigen
  // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  */

 public:

	SystemModel();
	~SystemModel();

  /*
   * Updates the model
   */
//  void init( double m, double d, double k );
//  void update( KDL::JntArray & tau_ );
//  void getStates( KDL::JntArray & q_m_, KDL::JntArray & qd_m_, KDL::JntArray & qdd_m_ );
//  void debug();

};

#endif /* SYSTEMMODEL_H_ */
