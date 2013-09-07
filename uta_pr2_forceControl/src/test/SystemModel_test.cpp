/*
 * SystemModel_test.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: Isura
 */

#include <SystemModel.h>

int main()
{
	KDL::JntArray  tau_;
	tau_.resize( 7 );

	tau_(0) = 0.5;
	tau_(1) = 0.5;
	tau_(2) = 0.5;
	tau_(3) = 0.5;
	tau_(4) = 0.5;
	tau_(5) = 0.5;
	tau_(6) = 0.5;

	SystemModel testClass( 10, 10, 10);

	testClass.update( tau_ );
	testClass.debug();

	testClass.update( tau_ );
	testClass.debug();

	testClass.update( tau_ );
	testClass.debug();

	testClass.update( tau_ );
	testClass.debug();

	return 0;
}
