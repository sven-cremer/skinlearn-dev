/*
 * pidController.h
 *
 *  Created on: Feb 22, 2015
 *      Author: Isura Ranatunga
 */

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

#include "uta_pr2_forceControl/neuroadptController.h"


namespace pr2_controller_ns{

class PR2PidControllerClass: public PR2NeuroadptControllerClass
{

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

#endif /* PIDCONTROLLER_H_ */
