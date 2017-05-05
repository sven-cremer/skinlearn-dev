/*
 * EigenConversions.h
 *
 *  Created on: Nov 11, 2015
 *      Author: sven
 */

#ifndef EIGENCONVERSIONS_H_
#define EIGENCONVERSIONS_H_


#include <Eigen/Core>
#include <kdl/jntarray.hpp>

namespace pr2_controller_ns{

typedef Eigen::Matrix<double, 6, 1> CartVec;
typedef Eigen::Matrix<double, 7, 1> PoseVec;

/////////////////////////////////////////////////////////////////////////
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
    Eigen::Vector3d rpy = mat.eulerAngles(0, 1, 2);	// The returned angles are in the ranges [0:pi]x[0:pi]x[-pi:pi]
    return rpy;
}
/////////////////////////////////////////////////////////////////////////
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

inline PoseVec
affine2PoseVec( Eigen::Affine3d a )
{
	PoseVec x;
	Eigen::Vector3d xyz(a.translation());
	Eigen::Quaterniond q(a.linear());

	x(0) = xyz(0);
	x(1) = xyz(1);
	x(2) = xyz(2);
	x(3) = q.x();
	x(4) = q.y();
	x(5) = q.z();
	x(6) = q.w();

    return x;
}
/////////////////////////////////////////////////////////////////////////
inline CartVec
PoseVec2CartVec( PoseVec a )
{
	CartVec x;
	Eigen::Quaterniond q( a(6),a(3),a(4),a(5) );
	Eigen::Vector3d rpy = quaternion2Euler( q );

	x(0) = a(0);
	x(1) = a(1);
	x(2) = a(2);
	x(3) = rpy(0);
	x(4) = rpy(1);
	x(5) = rpy(2);

    return x;
}

inline Eigen::Affine3d
PoseVec2Affine( PoseVec a )
{
	Eigen::Affine3d x;
	Eigen::Vector3d p(a(0),a(1),a(2));
	Eigen::Quaterniond q(a(6), a(3),a(4),a(5) );
	x = Eigen::Translation3d(p) * q;
    return x;
}
/////////////////////////////////////////////////////////////////////////
inline PoseVec
CartVec2PoseVec( CartVec a )
{
	PoseVec x;
	Eigen::Quaterniond q = euler2Quaternion( a(3),a(4),a(5) );

	x(0) = a(0);
	x(1) = a(1);
	x(2) = a(2);
	x(3) = q.x();
	x(4) = q.y();
	x(5) = q.z();
	x(6) = q.w();

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


//inline Eigen::MatrixXd
//JointKdl2Eigen( KDL::JntArray & joint_ )
//{
//	eigen_temp_joint(0) = joint_(0);
//	eigen_temp_joint(1) = joint_(1);
//	eigen_temp_joint(2) = joint_(2);
//	eigen_temp_joint(3) = joint_(3);
//	eigen_temp_joint(4) = joint_(4);
//	eigen_temp_joint(5) = joint_(5);
//	eigen_temp_joint(6) = joint_(6);
//
//	return eigen_temp_joint;
//}
//
//inline Eigen::MatrixXd
//JointVelKdl2Eigen( KDL::JntArrayVel & joint_ )
//{
//	eigen_temp_joint(0) = joint_.qdot(0);
//	eigen_temp_joint(1) = joint_.qdot(1);
//	eigen_temp_joint(2) = joint_.qdot(2);
//	eigen_temp_joint(3) = joint_.qdot(3);
//	eigen_temp_joint(4) = joint_.qdot(4);
//	eigen_temp_joint(5) = joint_.qdot(5);
//	eigen_temp_joint(6) = joint_.qdot(6);
//
//	return eigen_temp_joint;
//}

inline KDL::JntArray
JointEigen2Kdl( Eigen::VectorXd & joint )
{
	KDL::JntArray kdl_temp_joint_;
	kdl_temp_joint_.resize( 7 );		// TODO use num_Joints variable instead

	kdl_temp_joint_(0) = joint(0);
	kdl_temp_joint_(1) = joint(1);
	kdl_temp_joint_(2) = joint(2);
	kdl_temp_joint_(3) = joint(3);
	kdl_temp_joint_(4) = joint(4);
	kdl_temp_joint_(5) = joint(5);
	kdl_temp_joint_(6) = joint(6);

	return kdl_temp_joint_;
}



} // end namespace



#endif /* EIGENCONVERSIONS_H_ */
