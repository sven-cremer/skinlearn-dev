/*
 * math.h
 *
 *  Created on: Nov 25, 2014
 *      Author: sven
 */

#ifndef MATH_H_
#define MATH_H_

#include <Eigen/Core>

namespace csl
{
namespace math
{

// FIXME test this
Eigen::MatrixXd kroneckerProduct( Eigen::MatrixXd & A, Eigen::MatrixXd & B )
{
	  Eigen::MatrixXd kron(A.rows()*B.rows(), A.cols()*B.cols());

	  for (int i = 0; i < A.cols(); i++)
	  {
	      for (int j = 0; j < A.rows(); j++)
	      {
	    	  kron.block(i*B.rows(), j*B.cols(), B.rows(), B.cols()) =  A(i,j)*B;
	      }
	  }

	  return kron;
}

}
}



#endif /* MATH_H_ */
