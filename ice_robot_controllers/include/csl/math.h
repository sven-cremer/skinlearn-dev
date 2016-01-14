/*
 * math.h
 *
 *  Created on: Nov 25, 2014
 *      Author: sven
 */

#ifndef MATH_H_
#define MATH_H_

#include <Eigen/Core>
#include <KroneckerTensorProduct.h>

namespace csl
{
namespace math
{

// FIXME test this
inline Eigen::MatrixXd upperTriangularVector( Eigen::MatrixXd & A )
{
	  Eigen::MatrixXd upperTriVec(A.rows()*(A.rows()+1)/2, 1);

	  int index = 0;

	  for (int i = 0; i < A.cols(); i++)
	  {
	      for (int j = 0; j < A.rows(); j++)
	      {
	    	  if( j <= i )
	    	  {
	    		  upperTriVec(index,0) = A(i,j);
	    		  index+=1;
	    	  }
	      }
	  }

	  return upperTriVec;
}

}
}



#endif /* MATH_H_ */
