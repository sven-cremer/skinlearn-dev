/*
 * wristForceTorque.cpp
 *
 *  Created on: Aug 28, 2013
 *      Author: Isura Ranatunga
 */

#include <wristForceTorque.h>

WristForceTorque::WristForceTorque( )
{
	l_ft_handle_ = NULL;
	r_ft_handle_ = NULL;

	l_ft_samples = 0;
	r_ft_samples = 0;
}

void WristForceTorque::setLeftHandle( pr2_hardware_interface::ForceTorque* handle_ )
{
	l_ft_handle_ = handle_;
}

void WristForceTorque::setRightHandle( pr2_hardware_interface::ForceTorque* handle_ )
{
	r_ft_handle_ = handle_;
}

void WristForceTorque::setBias()
{

  std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
  l_ft_samples    = l_ftData_vector.size() - 1;
  l_ftBias.wrench = l_ftData_vector[l_ft_samples];

  std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
  r_ft_samples    = r_ftData_vector.size() - 1;
  r_ftBias.wrench = r_ftData_vector[r_ft_samples];

}

void WristForceTorque::update()
{

  std::vector<geometry_msgs::Wrench> l_ftData_vector = l_ft_handle_->state_.samples_;
  l_ft_samples    = l_ftData_vector.size() - 1;
  l_ftData.wrench = l_ftData_vector[l_ft_samples];

  std::vector<geometry_msgs::Wrench> r_ftData_vector = r_ft_handle_->state_.samples_;
  r_ft_samples    = r_ftData_vector.size() - 1;
  r_ftData.wrench = r_ftData_vector[r_ft_samples];

}

geometry_msgs::WrenchStamped WristForceTorque::getLeftData ( )
{
	return l_ftData;
}

geometry_msgs::WrenchStamped WristForceTorque::getRightData( )
{
	return r_ftData;
}

pr2_hardware_interface::ForceTorque* WristForceTorque::getLeftHandle ( )
{
	return l_ft_handle_;
}

pr2_hardware_interface::ForceTorque* WristForceTorque::getRightHandle( )
{
	return r_ft_handle_;
}
