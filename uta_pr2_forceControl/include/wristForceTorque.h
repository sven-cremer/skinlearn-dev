/*
 * wristForceTorque.h
 *
 *  Created on: Aug 28, 2013
 *      Author: Isura Ranatunga
 */

#include <pr2_hardware_interface/hardware_interface.h>
#include "geometry_msgs/WrenchStamped.h"

#ifndef WRISTFORCETORQUE_H_
#define WRISTFORCETORQUE_H_


class WristForceTorque
{

 private:

	pr2_hardware_interface::ForceTorque* l_ft_handle_;
	pr2_hardware_interface::ForceTorque* r_ft_handle_;

	int l_ft_samples;
	int r_ft_samples;

	geometry_msgs::WrenchStamped l_ftBias;
	geometry_msgs::WrenchStamped r_ftBias;

	geometry_msgs::WrenchStamped l_ftData;
	geometry_msgs::WrenchStamped r_ftData;

 public:

	void setLeftHandle ( pr2_hardware_interface::ForceTorque* handle_ );
	void setRightHandle( pr2_hardware_interface::ForceTorque* handle_ );
	void setBias();
	void update();
	geometry_msgs::WrenchStamped getLeftData ( );
	geometry_msgs::WrenchStamped getRightData( );
	pr2_hardware_interface::ForceTorque* getLeftHandle ( );
	pr2_hardware_interface::ForceTorque* getRightHandle( );

};


#endif /* WRISTFORCETORQUE_H_ */
