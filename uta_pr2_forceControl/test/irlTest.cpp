/*
 * irlTest.cpp
 *
 *  Created on: Nov 25, 2014
 *      Author: sven
 */

#include <csl/IrlModel.h>
#include <iostream>

int main(int argc, char **argv)
{
	csl::outer_loop::IrlModel irlModelObject;

	irlModelObject.init( 6, 2, 20, false );
	irlModelObject.setUpdateIrl();

	irlModelObject.update();
	irlModelObject.update();
	irlModelObject.update();
	irlModelObject.update();
	irlModelObject.update();
	irlModelObject.update();

	std::cout << "Worked!";

}
