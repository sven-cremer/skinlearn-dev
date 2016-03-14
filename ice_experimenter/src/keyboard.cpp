/*
 * keyboard.cpp
 *
 *  Created on: Jan 15, 2016
 *      Author: Sven Cremer
 */


//#include <boost/date_time.hpp>
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
#include <ice_msgs/setValue.h>
#include <ice_msgs/tactileCalibration.h>
#include <std_srvs/Empty.h>


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
	puts("Use 'c' to open calibration experiment menu");
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

void displayCalibrationExperimentMenu(int activeSensor, string dataFile, int curTrial, int numTrials, bool runningCalibration)
{
	// NN status
//	string tmp1 = "Status: " + (string)(controller_active ?  "ON" : "OFF");
//	string tmp2 = "Updating: " + (string)(controller_active ?  "YES" : "NO");
//	puts("---------------------------");
//	puts("   ice_controller     ");
//	puts(tmp1.c_str());
//	puts(tmp1.c_str());

	// Sensors
	puts("---------------------------");
	puts("MENU:   Calibration Experiment");
	printf("Active sensor: %i \n", activeSensor);
	if(!runningCalibration)
	{
		printf("Data file: %s + _#.rtp \n", dataFile.c_str());
		printf("Number of trials: %i \n", numTrials);
	}
	else
	{
		printf("Data file: %s + _%i.rtp \n", dataFile.c_str(),curTrial+1);
		printf("Running %i out of %i \n", curTrial+1, numTrials);
	}
	puts("---------------------------");
	puts("Tactile box layout");
	puts("         |          ");
	puts("  |------|------|   ");
	puts("  |             |   ");
	puts("  |      2      |   ");
	puts("                    ");
	puts("  1  >   ^   >  3 +y");
	puts("                    ");
	puts("         0          ");
	puts("        +x          ");
	puts("---------------------------");
	if(!runningCalibration)
	{
		puts("Use 'r' to enter number of calibration runs");
		puts("Use 'd' to change datafile name");
		puts("Use 's' to start calibration");
		puts("");
		puts("Use 'q' to quit and return to main menu");
		puts(" ");
	}
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
  ros::ServiceClient setTactileCalibration_srv_ = nh.serviceClient<ice_msgs::tactileCalibration>("/tactile/calibration");
  ros::ServiceClient publishExpData_srv_ = nh.serviceClient<std_srvs::Empty>("/pr2_adaptNeuroController/publishExpData");
  ros::ServiceClient status_srv_ = nh.serviceClient<ice_msgs::setBool>("/tactile/status");

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
      /******************************** calibration ****************************************/
      case 'c':
      {
    	  bool calibrationRunning = false;

    	  int activeSensor = 2;
    	  int nextActiveSensor = 2;
    	  string dataFile = "default";
    	  string dataDir = "~/test_rtp";		// TODO get package path
    	  string topic = "/pr2_adaptNeuroController/experimentDataB";

    	  int curTrial = 0;
    	  int numTrials = 4;

           bool stop_menu2 = false;
           while(!stop_menu2)
           {

        	 displayCalibrationExperimentMenu(activeSensor, dataFile, curTrial, numTrials, calibrationRunning);

             // Get the next event from the keyboard
        	 if(!calibrationRunning)
        	 {
				 if(read(kfd, &c, 1) < 0)
				 {
				   perror("read():");
				   exit(-1);
				 }

				 switch(c)
				 {
				 case 'd':
				 {
					 // Ask for file name
					 tcsetattr(kfd, TCSANOW, &cooked);         // Use old terminal settings
					 char cmd[100];
					 puts("*** Enter name of new data file ***");
					 cin.getline(cmd,100);
					 tcsetattr(kfd, TCSANOW, &raw);            // Use new terminal settings

					 dataFile = string(cmd);				   // TODO: add package path
				 }
				 break;
				 case 'r':
				 {
					 // Ask for number of trials
					 tcsetattr(kfd, TCSANOW, &cooked);         // Use old terminal settings
					 char cmd[100];
					 puts("*** Enter number of trials ***");
					 cin.getline(cmd,100);
					 tcsetattr(kfd, TCSANOW, &raw);            // Use new terminal settings

					 std::stringstream  linestream(cmd);
					 linestream >> numTrials;				   // TODO: check user input
				 }
				 break;
				 case 's':
				 {
					 calibrationRunning = true;
					 cout<<"Starting calibration!\n";
				 }
				 break;
				 case 'q':
					 stop_menu2 = true;
					 break;
				 default:
					 ROS_INFO_STREAM("Keycode not found: " << c);
					 break;
				 } //end switch

        	 }
        	 else	// Run calibration
             {
            	 tcsetattr(kfd, TCSANOW, &cooked);         // Use old terminal settings
            	 curTrial++;


            	 // Select sensor
            	 activeSensor = nextActiveSensor;
            	 ice_msgs::tactileCalibration tactileCalibration_msg;
     	 		 tactileCalibration_msg.request.start.position.x = 0.70;			// Green position
     	 		 tactileCalibration_msg.request.start.position.y = 0.35;
     	 		 tactileCalibration_msg.request.start.position.z = -0.1;
            	 tactileCalibration_msg.request.recordData = true;
            	 tactileCalibration_msg.request.activeSensor = activeSensor;
            	 switch(activeSensor)
            	 {
            	 	 case 2:	// Green -> blue (+x)
            	 		tactileCalibration_msg.request.distance = 0.2;
            	 		tactileCalibration_msg.request.time = 4;
            	 		nextActiveSensor = 3;	// Next sensor
            	 		break;
            	 	 case 3:	// Blue -> yellow (-y)
            	 		tactileCalibration_msg.request.start.position.x += 0.2;
            	 		tactileCalibration_msg.request.distance = 0.3;
            	 		tactileCalibration_msg.request.time = 6;
            	 		nextActiveSensor = 0;	// Next sensor
            	 		break;
            	 	 case 0:	// Yellow -> red (-x)
            	 		tactileCalibration_msg.request.start.position.x += 0.2;
            	 		tactileCalibration_msg.request.start.position.y -= 0.3;
            	 		tactileCalibration_msg.request.distance = 0.2;
            	 		tactileCalibration_msg.request.time = 4;
            	 		nextActiveSensor = 1;	// Next sensor
            	 		break;
            	 	 case 1:	// Red -> green (+y)
            	 		tactileCalibration_msg.request.start.position.y -= 0.3;
            	 		tactileCalibration_msg.request.distance = 0.3;
            	 		tactileCalibration_msg.request.time = 6;
            	 		nextActiveSensor = 2;	// Next sensor
            	 		break;
            	 	 default:
            	 		ROS_ERROR("Unknown active sensor!");
            	 }

            	 // Start calibration
            	 if (!setTactileCalibration_srv_.call(tactileCalibration_msg))
            	 {
            		 ROS_ERROR("Failed to call tactile calibration service!");
            	 }
            	 cout<<"Started calibration with sensor "<<activeSensor<<"\n";

            	 // Save rtp file
            	 std::ostringstream convert;
            	 convert << curTrial;
            	 std::string cmd1 = string("rostopic echo -p ") + topic.c_str() + string(" > ") + dataDir.c_str() + string("/") + dataFile.c_str() + string("_") + convert.str() + string(".rtp &");
            	 cout<<"$ "<<cmd1.c_str()<<"\n";
            	 system( cmd1.c_str() );

            	 // Check status
            	 cout<<"Waiting for calibration run to complete ... ";
            	 ice_msgs::setBool bool_msgs;
            	 bool_msgs.response.success = true;	// False when calibration is not longer runnign
            	 while(bool_msgs.response.success)
            	 {
            		 if(!status_srv_.call(bool_msgs))
                	 {
                		 ROS_ERROR("Failed to call status service!");
                	 }
            		 sleep(0.5);
            	 }
            	 cout<<" done!\n";

            	 // Publish data
            	 cout<<"Publishing data ...";
            	 std_srvs::Empty empty_msgs;
            	 if (!publishExpData_srv_.call(empty_msgs))
            	 {
            		 ROS_ERROR("Failed to call publishing data service!");
            	 }
            	 sleep(1.5);
            	 cout<<" done!\n";

            	 // Stop saving rtp file
            	 std::string cmd2 = string("pkill -9 -f ") + topic.c_str();
            	 cout<<"$ "<<cmd2.c_str()<<"\n";

            	 system( cmd2.c_str() );

            	 // Check if done
            	 if(curTrial >= numTrials)
            	 {
            		 calibrationRunning = false;
            		 curTrial = 0;
                	 cout<<"Completed calibration!\n";
            	 }
            	 tcsetattr(kfd, TCSANOW, &raw);            // Use new terminal settings
             }

           } // end while
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

