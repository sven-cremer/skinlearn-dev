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

private:

  Eigen::Matrix<double, 6, 1> eigen_vector_;
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

void updateEigen( Eigen::Matrix<double, 6, 1> & eigen_vector )
{
  eigen_vector_ = eigen_vector;
}

};

}



#endif /* OBJTEST_H_ */
