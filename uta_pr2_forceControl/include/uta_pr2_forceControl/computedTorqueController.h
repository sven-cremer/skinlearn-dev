/*
 * computedTorqueController.h
 *
 *  Created on: Feb 22, 2015
 *      Author: Isura Ranatunga
 */

#ifndef COMPUTEDTORQUECONTROLLER_H_
#define COMPUTEDTORQUECONTROLLER_H_

#include "uta_pr2_forceControl/neuroadptController.h"

// RBDL
#include <rbdl/rbdl.h>
#include <rbdl/rbdl_utils.h>
#include "rbdl_urdfreader.h"

namespace pr2_controller_ns{

class PR2ComputedTorqueControllerClass: public PR2NeuroadptControllerClass
{

	RigidBodyDynamics::Model m_model;

private:

  bool paramUpdate( neuroadaptive_msgs::controllerParamUpdate::Request  & req ,
                    neuroadaptive_msgs::controllerParamUpdate::Response & resp );

  bool save( neuroadaptive_msgs::saveControllerData::Request & req,
  	         neuroadaptive_msgs::saveControllerData::Response& resp );

  bool publish( std_srvs::Empty::Request & req,
                std_srvs::Empty::Response& resp );

  bool capture(std_srvs::Empty::Request& req,
               std_srvs::Empty::Response& resp);

  bool saveControllerData( neuroadaptive_msgs::saveControllerData::Request&  req,
                           neuroadaptive_msgs::saveControllerData::Response& resp );

  void bufferData( double & dt );


public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

public:

  bool init(pr2_mechanism_model::RobotState *robot,
            ros::NodeHandle &n);
  void starting();
  void update();
  void stopping();

};

}


#endif /* COMPUTEDTORQUECONTROLLER_H_ */
