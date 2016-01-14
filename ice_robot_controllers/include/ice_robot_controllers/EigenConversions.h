/*
 * EigenConversions.h
 *
 *  Created on: Nov 11, 2015
 *      Author: sven
 */

#ifndef EIGENCONVERSIONS_H_
#define EIGENCONVERSIONS_H_


#include <Eigen/Core>

namespace pr2_controller_ns{

typedef Eigen::Matrix<double, 6, 1> CartVec;

inline Eigen::Quaterniond
euler2Quaternion( const double roll,
                  const double pitch,
                  const double yaw )
{
    Eigen::AngleAxisd rollAngle(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw, Eigen::Vector3d::UnitZ());

    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    return q;
}

inline Eigen::Vector3d
quaternion2Euler( Eigen::Quaterniond q )
{
	Eigen::Matrix3d mat = q.matrix();
    Eigen::Vector3d rpy = mat.eulerAngles(0, 1, 2);
    return rpy;
}

inline CartVec
affine2CartVec( Eigen::Affine3d a )
{
	CartVec x;
	Eigen::Vector3d xyz(a.translation());
	Eigen::Quaterniond q(a.linear());
	Eigen::Vector3d rpy = quaternion2Euler( q );

	x(0) = xyz(0);
	x(1) = xyz(1);
	x(2) = xyz(2);
	x(3) = rpy(0);
	x(4) = rpy(1);
	x(5) = rpy(2);

    return x;
}

inline Eigen::Affine3d
CartVec2Affine( CartVec a )
{
	Eigen::Affine3d x;
	Eigen::Vector3d p(a(0),a(1),a(2));
	Eigen::Quaterniond q = euler2Quaternion( a(3),a(4),a(5) );
	x = Eigen::Translation3d(p) * q;
    return x;
}

}



#endif /* EIGENCONVERSIONS_H_ */
