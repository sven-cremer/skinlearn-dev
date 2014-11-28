/*
 * kronTest.cc
 *
 *  Created on: Nov 28, 2014
 *      Author: sven
 */

#include <csl/math.h>
#include <iostream>

int main(int argc, char **argv)
{
	int test = 0;

	Eigen::MatrixXd A;
	Eigen::MatrixXd B;

	A.resize( 2, 2 );
	B.resize( 2, 2 );

	A << 1, 2,
	     3, 4;

	B << 0, 5,
	     6, 7;

	std::cout << Eigen::kroneckerProduct(A,B).eval();

//	Should be:
//	 0  5  0 10
//	 6  7 12 14
//	 0 15  0 20
//	18 21 24 28

	std::cout << "Worked!";

}



