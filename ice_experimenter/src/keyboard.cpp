/*
 * keyboard.cpp
 *
 *  Created on: Jan 15, 2016
 *      Author: Sven Cremer
 */


#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH
#include <termios.h>
#include <signal.h>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// Messages
#include <ice_msgs/setBool.h>
#include <ice_msgs/getNNweights.h>
#include <ice_msgs/setNNweights.h>
#include <ice_msgs/getState.h>
#include <ice_msgs/twoLayerNN.h>


using namespace std;

static bool controller_active = false;
static bool updating_weights = true;

void displayMainMenu()
{
	string tmp1 = "Status: " + (string)(controller_active ?  "ON" : "OFF");
	puts("---------------------------");
	puts("MENU:   ice_controller     ");
	puts(tmp1.c_str());
	puts("---------------------------");
	puts("Use '1' to turn controller ON");
	puts("Use '2' to turn controller OFF");
	puts("Use 'i' to initialize robot");
	puts("Use 'w' to open NN weights menu");
	puts("Use 'r' to open reference trajectory menu");
	puts(" ");
	puts("Use '-' to set variables");
	puts(" ");
	puts("Use 'q' to quit");
	puts(" ");
}

void displayNNweightsMenu()
{
	string tmp1 = "Updating: " + (string)(updating_weights ?  "YES" : "NO");
	puts("---------------------------");
	puts("MENU:   NN weights     ");
	puts(tmp1.c_str());
	puts("---------------------------");
	puts("Use 't' to toggle weight updating.");
	puts("Use 's' to save NN weights.");
	puts("Use 'l' to load NN weights.");
	puts(" ");
	puts("Use 'q' to quit and return to main menu");
	puts(" ");
}

//void displayRefTrajMenu()
//{
//	string tmp1 = "Updating: " + (string)(controller_active ?  "YES" : "NO");
//	puts("---------------------------");
//	puts("MENU:   Reference trajectory");
//	printf("Selected: %i \n", 1);
//	puts("---------------------------");
//	puts("Use '1' to switch to ?.");
//	puts("Use '2' to switch to ?.");
//	puts("Use '3' to switch to ?.");
//	puts(" ");
//	puts("Use 'q' to quit and return to main menu");
//	puts(" ");
//}

int kfd = 0;
struct termios cooked, raw;

void spin_function()
{
  ros::spin();
}

void quit(int sig)
{
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}


/***********************************************************************************************************************
int main(int argc, char **argv)
program entry point, loop for keyboard commands
***********************************************************************************************************************/
int main(int argc, char** argv)
{
  ros::init(argc, argv, "keyboard_interface", ros::init_options::NoSigintHandler);

  ros::NodeHandle nh;

  // ROS service clients
  ros::ServiceClient toggle_updateWeights_srv_ = nh.serviceClient<ice_msgs::setBool>("/pr2_adaptNeuroController/updateNNweights");
  ros::ServiceClient setNNWeights_srv_ = nh.serviceClient<ice_msgs::setNNweights>("/pr2_adaptNeuroController/setNNweights");
  ros::ServiceClient getNNWeights_srv_ = nh.serviceClient<ice_msgs::getNNweights>("/pr2_adaptNeuroController/getNNweights");

  // ROS messages
  ice_msgs::setBool setBool_msgs_;


  signal(SIGINT,quit);

  boost::thread spin_thread(boost::bind(&spin_function));

  // get the console in raw mode
  tcgetattr(kfd, &cooked);							// grab old terminal i/o settings "cooked"
  memcpy(&raw, &cooked, sizeof(struct termios));	// copy to new setting "raw"
  raw.c_lflag &= ~(ICANON | ECHO);					// noncanonical mode (get single key presses); do not print input
  // Setting a new line, then end of file
  //raw.c_cc[VEOL] = 1;								// (0, NUL) Additional end-of-line character (EOL). Recognized when ICANON is set.
  //raw.c_cc[VEOF] = 2;								// since ICANON is not set this should have no effect
  tcsetattr(kfd, TCSANOW, &raw);					// sets the new terminal settings


  char c;

  puts("Reading from keyboard");
  bool stop = false;
  while(!stop)
  {
    displayMainMenu();

    // get the next event from the keyboard
    if(read(kfd, &c, 1) < 0)
    {
      perror("read():");
      exit(-1);
    }

    switch(c)
    {
      /******************************** Manger *******************************************/
      case '1':
      {
    	  // controller on
    	  controller_active = true;
    	  break;
      }
      case '2':
      {
    	  // controller off
    	  controller_active = false;
    	  break;
      }
      case 'i':
      {
    	  // init robot
    	  break;
      }
      /******************************** NN weights ****************************************/
      case 'w':
      {
        bool stop_menu2 = false;
        string fname;

        while(!stop_menu2)
        {

          displayNNweightsMenu();

          // get the next event from the keyboard
          if(read(kfd, &c, 1) < 0)
          {
            perror("read():");
            exit(-1);
          }

          switch(c)
          {
		  case 't':
		  {
			  setBool_msgs_.request.variable = !updating_weights;

			  if (toggle_updateWeights_srv_.call(setBool_msgs_))
			  {
				  updating_weights = setBool_msgs_.request.variable;
			  }
			  else
			  {
				  ROS_ERROR("Failed to call service!");
			  }
		  }
			  break;
		  case 's':
		  {
			  // Get weights
			  ice_msgs::getNNweights getNNWeights_msg_;
			  if (getNNWeights_srv_.call(getNNWeights_msg_))
			  {
				  // Save message to file
				  tcsetattr(kfd, TCSANOW, &cooked);         // Use old terminal settings
				  char cmd[100];
				  puts("*** Enter name of new rosbag file ***");
				  cin.getline(cmd,100);
				  tcsetattr(kfd, TCSANOW, &raw);            // Use new terminal settings

				  fname = string(cmd)+".bag";				// TODO: add package path

				  rosbag::Bag bag;

				  bag.open(fname.c_str(), rosbag::bagmode::Write);

				  //TODO:
				  bag.write("network", ros::Time::now(), getNNWeights_msg_.response.net);

				  bag.close();

			  }
			  else
			  {
				  ROS_ERROR("Failed to call service!");
			  }
		  }
			  break;
		  case 'l':
		  {
			  // Ask for file name
			  tcsetattr(kfd, TCSANOW, &cooked);         // Use old terminal settings
			  char cmd[100];
			  puts("*** Enter name of rosbag file ***");
			  cin.getline(cmd,100);
			  tcsetattr(kfd, TCSANOW, &raw);            // Use new terminal settings

			  fname = string(cmd)+".bag";				// TODO: add package path

			  // Load NN from file
			  rosbag::Bag bag;
			  bag.open(fname.c_str(), rosbag::bagmode::Read);

			  std::vector<std::string> topics;
			  topics.push_back(std::string("network"));

			  ice_msgs::setNNweights setNNWeights_msg_;

			  rosbag::View view(bag, rosbag::TopicQuery(topics));

			  foreach(rosbag::MessageInstance const m, view)
			  {
				  ice_msgs::twoLayerNN::ConstPtr w = m.instantiate<ice_msgs::twoLayerNN>();
				  if (w != NULL)
				  {
//					  std::cout<<"num_Inputs:  "<<w->num_Inputs<<"\n";
//					  std::cout<<"num_Hidden:  "<<w->num_Hidden<<"\n";
//					  std::cout<<"num_Outputs: "<<w->num_Outputs<<"\n";
//					  std::cout<<"V: "<<w->V<<"\n";
//					  std::cout<<"-------------\n";
//					  std::cout<<"W: "<<w->W<<"\n";
					  setNNWeights_msg_.request.net = *w;
				  }
			  }

			  bag.close();

			  // Call service
			  if (!setNNWeights_srv_.call(setNNWeights_msg_))
			  {
				  ROS_ERROR("Failed to call service!");
			  }

			  if(!setNNWeights_msg_.response.success)
			  {
				  ROS_ERROR("Failed to set weights!");
			  }

		  }
			  break;
		  case 'q':
			  stop_menu2 = true;
			  break;
		  default:
			  ROS_INFO_STREAM("Keycode not found: " << c);
			  break;
          } //end switch
        } // end while
        break;
      }
      /******************************** reference ****************************************/
      case 'r':
      {
    	  break;
      }
      /******************************** QUIT ****************************************/
      case 'q':
      {
        stop = true;
        break;
      }
      default:
      {
          ROS_INFO_STREAM("Keycode not found: " << c);
        break;
      }
    }
  }


  puts("Shutting down ...");

 // TODO: Turn off controller

  tcsetattr(kfd, TCSANOW, &cooked);	// Revert to old terminal settings

  ros::shutdown();
  spin_thread.join();
  return(0);
}

