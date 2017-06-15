/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

// Author: Stuart Glaser
// Modified by Sven Cremer (June 15, 2017)

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <ros/ros.h>
#include <tf/tf.h>

//#include <angles/angles.h>

class JTCartesianController
{
public:
  // Ensure 128-bit alignment for Eigen
  // See also http://eigen.tuxfamily.org/dox/StructHavingEigenMembers.html
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
private:
  enum { Joints = 7 };
  typedef Eigen::Matrix<double, Joints, 1> JointVec;
  typedef Eigen::Matrix<double, 6, 1> CartVec;
  typedef Eigen::Matrix<double, 6, Joints> Jacobian;

  ros::NodeHandle node_;

public:
  JTCartesianController(ros::NodeHandle &n);
  ~JTCartesianController();

  Eigen::Affine3d x_desi_filtered_;

  Eigen::Matrix<double,6,1> Kp, Kd;

  double pose_command_filter_;
  double vel_saturation_trans_, vel_saturation_rot_;
  double joint_vel_filter_;

  int loop_count_;

  void starting(Eigen::Affine3d x0)
  {
	  x_desi_filtered_ = x0;
  }

  void update(Eigen::Affine3d & x,
  	          Eigen::VectorXd & xdot,
  	          Eigen::Affine3d & x_desi_,
              Eigen::VectorXd & fc);

//  void setGains(const std_msgs::Float64MultiArray::ConstPtr &msg)
//  {
//    if (msg->data.size() >= 6)
//      for (size_t i = 0; i < 6; ++i)
//        Kp[i] = msg->data[i];
//    if (msg->data.size() == 12)
//      for (size_t i = 0; i < 6; ++i)
//        Kd[i] = msg->data[6+i];
//
//    ROS_INFO("New gains: %.1lf, %.1lf, %.1lf %.1lf, %.1lf, %.1lf",
//             Kp[0], Kp[1], Kp[2], Kp[3], Kp[4], Kp[5]);
//  }

};


JTCartesianController::JTCartesianController(ros::NodeHandle &n)
{
	node_ = n;

  // Cartesian gains
  double kp_trans, kd_trans, kp_rot, kd_rot;
  if (!node_.getParam("cart_gains/trans/p", kp_trans) ||
      !node_.getParam("cart_gains/trans/d", kd_trans))
  {
    ROS_ERROR("P and D translational gains not specified (namespace: %s)", node_.getNamespace().c_str());
    kp_trans = 800.0;
    kd_trans = 80.0;
  }
  if (!node_.getParam("cart_gains/rot/p", kp_rot) ||
      !node_.getParam("cart_gains/rot/d", kd_rot))
  {
    ROS_ERROR("P and D rotational gains not specified (namespace: %s)", node_.getNamespace().c_str());
    kp_rot = 15.0;
    kd_rot = 1.2;
  }
  Kp << kp_trans, kp_trans, kp_trans,  kp_rot, kp_rot, kp_rot;
  Kd << kd_trans, kd_trans, kd_trans,  kd_rot, kd_rot, kd_rot;

  node_.param("pose_command_filter", pose_command_filter_, 1.0);

  // Velocity saturation
  node_.param("vel_saturation_trans", vel_saturation_trans_, 0.0);
  node_.param("vel_saturation_rot", vel_saturation_rot_, 0.0);

  node_.param("joint_vel_filter", joint_vel_filter_, 1.0);

  loop_count_ = 0;
}

JTCartesianController::~JTCartesianController()
{
}


static void computePoseError(const Eigen::Affine3d &xact, const Eigen::Affine3d &xdes, Eigen::Matrix<double,6,1> &err)
{
  err.head<3>() = xact.translation() - xdes.translation();
  err.tail<3>()   = 0.5 * (xdes.linear().col(0).cross(xact.linear().col(0)) +
                          xdes.linear().col(1).cross(xact.linear().col(1)) +
                          xdes.linear().col(2).cross(xact.linear().col(2)));
}

void JTCartesianController::update(Eigen::Affine3d & x,
								   Eigen::VectorXd & xdot,
								   Eigen::Affine3d & x_desi_,
                                   Eigen::VectorXd & fc)
{

  // ======== Controls to the current pose setpoint

  {
    Eigen::Vector3d p0(x_desi_filtered_.translation());
    Eigen::Vector3d p1(x_desi_.translation());
    Eigen::Quaterniond q0(x_desi_filtered_.linear());
    Eigen::Quaterniond q1(x_desi_.linear());
    q0.normalize();
    q1.normalize();

    tf::Quaternion tf_q0(q0.x(), q0.y(), q0.z(), q0.w());
    tf::Quaternion tf_q1(q1.x(), q1.y(), q1.z(), q1.w());
    tf::Quaternion tf_q = tf_q0.slerp(tf_q1, pose_command_filter_);

    Eigen::Vector3d p = p0 + pose_command_filter_ * (p1 - p0);
    //Eigen::Quaterniond q = q0.slerp(pose_command_filter_, q1);
    Eigen::Quaterniond q(tf_q.w(), tf_q.x(), tf_q.y(), tf_q.z());
    //x_desi_filtered_ = q * Eigen::Translation3d(p);
    x_desi_filtered_ = Eigen::Translation3d(p) * q;
  }
  CartVec x_err;
  //computePoseError(x, x_desi_, x_err);
  computePoseError(x, x_desi_filtered_, x_err);

  CartVec xdot_desi = (Kp.array() / Kd.array()) * x_err.array() * -1.0;

  // Caps the cartesian velocity
  if (vel_saturation_trans_ > 0.0)
  {
    if (fabs(xdot_desi.head<3>().norm()) > vel_saturation_trans_)
      xdot_desi.head<3>() *= (vel_saturation_trans_ / xdot_desi.head<3>().norm());
  }
  if (vel_saturation_rot_ > 0.0)
  {
    if (fabs(xdot_desi.tail<3>().norm()) > vel_saturation_rot_)
      xdot_desi.tail<3>() *= (vel_saturation_rot_ / xdot_desi.tail<3>().norm());
  }

  fc = Kd.array() * (xdot_desi - xdot).array();

  loop_count_++;

}

