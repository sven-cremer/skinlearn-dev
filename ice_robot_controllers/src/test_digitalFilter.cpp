/*
 * test_digitalFilter.cpp
 *
 *  Created on: Jul 10, 2017
 *      Author: Sven Cremer
 */

#include <ice_robot_controllers/digitalFilter.h>
#include <iostream>
#include <sys/time.h>

using namespace pr2_controller_ns;

int main(int argc, char** argv)
{
	int order = 2;
	bool isIIR = false;

	Eigen::VectorXd a;
	Eigen::VectorXd b;

	// Band-pass, 1st order chebychev
	// 5-100Hz, fs=1000Hz
	a.resize(order+1);
	b.resize(order+1);
	a << 1.0,-1.0526,0.0634;
	b <<0.4683, 0, -0.4683;

	digitalFilter d1, d2;
	d1.init(order,isIIR,a,b);
	d2.init(order,isIIR,a,b);

	// Input vector
	int N = 10;
	srand ( 0 );
	Eigen::VectorXd x;
	x.setRandom(N);

	// Filter
	Eigen::VectorXd x1, x2;
	x1.resize(N);
	x2.resize(N);

	timespec t1, t2;
	int diff;

	// Filter version 1
	clock_gettime(CLOCK_REALTIME, &t1);
	for(int i=0;i<N;i++)
	{
		x1(i) = d1.getNextFilteredValue(x(i));
	}
	clock_gettime(CLOCK_REALTIME, &t2);
	diff = (t2.tv_sec - t1.tv_sec) * 1000000000 + (t2.tv_nsec - t1.tv_nsec);
    std::cout << "Original Run time: " << diff << "\t(nsec)\n";

	// Filter version 2
	clock_gettime(CLOCK_REALTIME, &t1);
	for(int i=0;i<N;i++)
	{
		x2(i) = d2.getNextFilteredValueNew(x(i));
	}
	clock_gettime(CLOCK_REALTIME, &t2);
	diff = (t2.tv_sec - t1.tv_sec) * 1000000000 + (t2.tv_nsec - t1.tv_nsec);
    std::cout << "New Run time:     " << diff << "\t(nsec)\n";

    // Display result
    if(N<100)
    {
    	std::cout<<"x  = "<< x.transpose()  << "\n---\n";
    	std::cout<<"x1 = "<< x1.transpose() << "\n---\n";
    	std::cout<<"x2 = "<< x2.transpose() << "\n---\n";
    }
    else
    {
    	std::cout<< "sum(x2-x1) = " << (x2-x1).sum()  << "\n---\n";
    }

	return 0;

}
