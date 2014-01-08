/*
 * objTest.h
 *
 *  Created on: Jan 8, 2014
 *      Author: pr2admin
 */

#pragma once

#include <Eigen/Core>
#include <Eigen/Eigenvalues>

#ifndef OBJTEST_H_
#define OBJTEST_H_

namespace test
{

class TestObjectClass
{

public:


  enum {
          Inputs = 35
  }; // n Size of the inputs
  enum {
          Outputs = 7
  }; // m Size of the outputs
  enum {
          Hidden = 10
  }; // l Size of the hidden layer
  enum {
          Error = 7
  }; // filtered error

  // Declare the number of joints.
  enum { Joints = 7 };

  // Define the joint/cart vector types accordingly (using a fixed
  // size to avoid dynamic allocations and make the code realtime safe).
  typedef Eigen::Matrix<double, Joints, Joints>  SystemMatrix;
  typedef Eigen::Matrix<double, Joints, 1>       SystemVector;

private:

  SystemVector eigen_vector_;
  double test_double_;

public:
  TestObjectClass()
  {
    eigen_vector_.Zero();
    test_double_ = 0;
  }

  ~TestObjectClass()
  {
  }

void updateDouble( double testDouble )
{
  test_double_ = testDouble;
}

void updateEigen( SystemVector & eigen_vector )
{
  eigen_vector_ = eigen_vector;
}

};

}



#endif /* OBJTEST_H_ */
