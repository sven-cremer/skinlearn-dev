/*
 * SystemModel.cpp
 *
 *  Created on: Sep 7, 2013
 *      Author: Isura
 */

#include <iostream>
#include "SystemModel.h"

SystemModel::SystemModel()
{
//	init( 10, 10, 10 );
}

SystemModel::~SystemModel()
{
	// TODO Auto-generated destructor stub
}

/*
void SystemModel::init( double m, double d, double k )
{
	Mm << m, 0, 0, 0, 0, 0, 0,
		  0, m, 0, 0, 0, 0, 0,
		  0, 0, m, 0, 0, 0, 0,
		  0, 0, 0, m, 0, 0, 0,
		  0, 0, 0, 0, m, 0, 0,
		  0, 0, 0, 0, 0, m, 0,
		  0, 0, 0, 0, 0, 0, m;

	Dm << d, 0, 0, 0, 0, 0, 0,
		  0, d, 0, 0, 0, 0, 0,
		  0, 0, d, 0, 0, 0, 0,
		  0, 0, 0, d, 0, 0, 0,
		  0, 0, 0, 0, d, 0, 0,
		  0, 0, 0, 0, 0, d, 0,
		  0, 0, 0, 0, 0, 0, d;

	Km << k, 0, 0, 0, 0, 0, 0,
		  0, k, 0, 0, 0, 0, 0,
		  0, 0, k, 0, 0, 0, 0,
		  0, 0, 0, k, 0, 0, 0,
		  0, 0, 0, 0, k, 0, 0,
		  0, 0, 0, 0, 0, k, 0,
		  0, 0, 0, 0, 0, 0, k;

	q_m   << 0, 0, 0, 0, 0, 0, 0 ;
	qd_m  << 0, 0, 0, 0, 0, 0, 0 ;
	qdd_m << 0, 0, 0, 0, 0, 0, 0 ;

	t_h   << 0, 0, 0, 0, 0, 0, 0 ;

	MmInv = Mm;

	delT  = 0.001;
}

void SystemModel::update( KDL::JntArray & tau_human )
{
	t_h(0) = tau_human(0);
	t_h(1) = tau_human(1);
	t_h(2) = tau_human(2);
	t_h(3) = tau_human(3);
	t_h(4) = tau_human(4);
	t_h(5) = tau_human(5);
	t_h(6) = tau_human(6);

	q_m   = q_m + delT*qd_m;
	qd_m  = qd_m + delT*qdd_m;
	qdd_m = MmInv*( t_h - Dm*qd_m - Km*q_m );
}

void SystemModel::getStates( KDL::JntArray & q_m_, KDL::JntArray & qd_m_, KDL::JntArray & qdd_m_ )
{
	q_m_(0)   = q_m(0);
	q_m_(1)   = q_m(1);
	q_m_(2)   = q_m(2);
	q_m_(3)   = q_m(3);
	q_m_(4)   = q_m(4);
	q_m_(5)   = q_m(5);
	q_m_(6)   = q_m(6);

	qd_m_(0)  = qd_m(0);
	qd_m_(1)  = qd_m(1);
	qd_m_(2)  = qd_m(2);
	qd_m_(3)  = qd_m(3);
	qd_m_(4)  = qd_m(4);
	qd_m_(5)  = qd_m(5);
	qd_m_(6)  = qd_m(6);

	qdd_m_(0) = qdd_m(0);
	qdd_m_(1) = qdd_m(1);
	qdd_m_(2) = qdd_m(2);
	qdd_m_(3) = qdd_m(3);
	qdd_m_(4) = qdd_m(4);
	qdd_m_(5) = qdd_m(5);
	qdd_m_(6) = qdd_m(6);
}

void SystemModel::debug()
{
	std::cout << "\n Mm    :\n" << Mm;
	std::cout << "\n Dm    :\n" << Dm;
	std::cout << "\n Km    :\n" << Km;

	std::cout << "\n q_m   :\n" << q_m.transpose();
	std::cout << "\n qd_m  :\n" << qd_m.transpose();
	std::cout << "\n qdd_m :\n" << qdd_m.transpose();
}
*/
