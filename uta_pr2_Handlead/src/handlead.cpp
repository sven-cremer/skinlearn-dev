/**
*
* handlead: creates a ROS node that allows the PR2 to be lead around by the hand.
*
* This node listens to a /ft/r_gripper_motor and /ft/l_gripper_motor topics and sends messages to the /base_controller/command topic.
*
* @author Isura Ranatunga, University of Texas at Arlington, Copyright (C) 2013.
* @contact isura.ranatunga@mavs.uta.edu
* @created 01/12/2013
* @modified 01/12/2013
*
*/

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Joy.h>

class pr2_handlead
{
 public:
          pr2_handlead()
          {
            // create ROS publisher
            cmd_vel = node.advertise<geometry_msgs::Twist>("base_controller/command", 10);
	    // create ROS subscriber
            joy_sub = node.subscribe("joy", 10, &pr2_handlead::joy_callback, this);
            rFT_sub = node.subscribe("ft/r_gripper_motor", 10, &pr2_handlead::rFT_callback, this);
	    lFT_sub = node.subscribe("ft/l_gripper_motor", 10, &pr2_handlead::lFT_callback, this);

	    deadmanSwitch = false;
	    zeroForce = false;

	    // TODO these values change depending on the gripper position/orientation
	    // need to come up with better way to keep these zeroed

	    rdefaultWrench.force.x = 3.959748;
	    rdefaultWrench.force.y = -3.38058;
	    rdefaultWrench.force.z = -10.543229;
	    rdefaultWrench.torque.x = -0.446074;
	    rdefaultWrench.torque.y = -0.475492;
	    rdefaultWrench.torque.z = 0.113609;

	    // TODO change these to correct left FT defaults
	    ldefaultWrench.force.x = 3.959748;
	    ldefaultWrench.force.y = -3.38058;
	    ldefaultWrench.force.z = -10.543229;
	    ldefaultWrench.torque.x = -0.446074;
	    ldefaultWrench.torque.y = -0.475492;
	    ldefaultWrench.torque.z = 0.113609;

            ROS_INFO("PR2 Hand-lead demo started!");
          }
 private:

          void joy_callback(const sensor_msgs::Joy& joy)
          {
		deadmanSwitch = false;
		zeroForce = false;

		if(joy.axes.at(10) < 0.0)
		{
			deadmanSwitch = true;
		}

		if(joy.axes.at(11) < 0.0)
		{
			zeroForce = true;
		}

		//ROS_INFO("Joy deadman switch: %f | %d", joy.axes.at(10), deadmanSwitch);
          }  

          void rFT_callback(const geometry_msgs::WrenchStamped& ftmsg)
          {
            // create twist and wrench msg
            geometry_msgs::Twist twist;	    
	    geometry_msgs::WrenchStamped ft;

            ft = ftmsg;

	    if(zeroForce == true){

		rdefaultWrench.force.x = ft.wrench.force.x;
	    	rdefaultWrench.force.y = ft.wrench.force.y;
	    	rdefaultWrench.force.z = ft.wrench.force.z;
	    	rdefaultWrench.torque.x = ft.wrench.torque.x;
	    	rdefaultWrench.torque.y = ft.wrench.torque.y;
	    	rdefaultWrench.torque.z = ft.wrench.torque.z;

		}

	    //subtract default wrench
	    ft.wrench.force.x  = ft.wrench.force.x - rdefaultWrench.force.x;
	    ft.wrench.force.y = ft.wrench.force.y - rdefaultWrench.force.y;
	    ft.wrench.force.z = ft.wrench.force.z - rdefaultWrench.force.z; 
	    ft.wrench.torque.x = ft.wrench.torque.x - rdefaultWrench.torque.x;
	    ft.wrench.torque.y = ft.wrench.torque.y - rdefaultWrench.torque.y;
	    ft.wrench.torque.z = ft.wrench.torque.z - rdefaultWrench.torque.z;

	    ROS_INFO("Force: %f %f %f | Torque: %f %f %f", ft.wrench.force.x, ft.wrench.force.y, ft.wrench.force.z, ft.wrench.torque.x, ft.wrench.torque.y, ft.wrench.torque.z);

	    if(deadmanSwitch == true){

		    // linear
		    twist.linear.x = ft.wrench.force.z/10;
		    twist.linear.y = 0;
		    //twist.linear.y = ft.wrench.force.y/10; // TODO this does not work that well
		    twist.linear.z = 0;
		    // angular
		    twist.angular.x = 0;
		    twist.angular.y = 0;
		    twist.angular.z = 0;

	    }else{

		    // linear
		    twist.linear.x = 0;
		    twist.linear.y = 0;
		    twist.linear.z = 0;
		    // angular
		    twist.angular.x = 0;
		    twist.angular.y = 0;
		    twist.angular.z = 0;

	    }

            // send twist
            cmd_vel.publish(twist);
	    
          }

          void lFT_callback(const geometry_msgs::WrenchStamped& ftmsg)
          {
            // create twist msg
            geometry_msgs::Twist twist;

	    //ROS_INFO("Force: %f %f %f | Torque: %f %f %f", ft.wrench.force.x, ft.wrench.force.y, ft.wrench.force.z, ft.wrench.torque.x, ft.wrench.torque.y, ft.wrench.torque.z);
           
          }

  ros::NodeHandle node;
  ros::Publisher cmd_vel;
  ros::Subscriber joy_sub, rFT_sub, lFT_sub;

  //deadman switch
  bool deadmanSwitch;

  //zero force switch
  bool zeroForce;

  //default force values
  geometry_msgs::Wrench rdefaultWrench;
  geometry_msgs::Wrench ldefaultWrench;

};

int main(int argc, char **argv)
{
  // start pr2_handlead node
  ros::init(argc, argv, "pr2_handlead");
  pr2_handlead handlead;
  ros::spin();
}
