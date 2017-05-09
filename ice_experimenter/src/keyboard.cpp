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
#include <Eigen/Core>

// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <sound_play/sound_play.h>

// PR2 utilities
#include <apc_robot/pr2_manager.h>
#include <apc_robot/apc_arms_cartesian.h>
#include <trajectory_generator/trajectoryGenerator.h>

// Messages
#include <ice_msgs/setBool.h>
#include <ice_msgs/getNNweights.h>
#include <ice_msgs/setNNweights.h>
#include <ice_msgs/getState.h>
#include <ice_msgs/twoLayerNN.h>
#include <ice_msgs/setValue.h>
#include <ice_msgs/tactileCalibration.h>
#include <ice_msgs/tactileFilterWeights.h>
#include <std_srvs/Empty.h>
#include <ice_msgs/setTrajectory.h>
#include <geometry_msgs/PoseStamped.h>


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
	puts("Use '0' to change controllers");
	puts("Use '1' to turn controller ON");
	puts("Use '2' to turn controller OFF");
	puts("Use 'i' to initialize robot");
	puts("Use 'o/p' to open/close LEFT gripper");
	puts("Use 'j/k' to open/close RIGHT gripper");
	puts(" ");
	puts("Use 'u/h' to lift torso up/down");
	puts(" ");
	puts("Use 'w' to open NN weights menu");
	puts("Use 'r' to open NN reference trajectory menu");
	puts("Use 's' to start calibration experiment");
	puts("Use 't' to test controller");
	puts(" ");
	puts("Use '-' to set variables");
	puts(" ");
	puts("Use 'q' to quit");
	puts(" ");
}

void displayControllerMenu()
{
	puts("---------------------------");
	puts("MENU:   Controllers     ");
	puts("---------------------------");
	puts("Use '1' left default / NN");
	puts("Use '2' left default / JT Cartesian");
	puts("Use '3' left + right default / JT Cartesian");
	puts(" ");
	puts("Use 'q' to quit and return to main menu");
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
		puts("Use 'c' to capture data");
		puts("Use 'p' to publish data");
		puts("Use 'f' to fix filter weights");
		puts("Use 'u' to use uncalibrated filter weights");
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

void getKey(char &c)
{
	// get the next event from the keyboard
	if(read(kfd, &c, 1) < 0)
	{
		perror("read():");
		exit(-1);
	}
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

  PR2Manager pr2manager;
  double torsoHeight = 0.05;
  double deltaTorsoHeight = 0.01;

  // Trajectory settings
  geometry_msgs::Point origin;
  origin.x = 0.38;
  origin.y = 0.15;
  origin.z = -0.10;
  TrajectoryGenerator tg;
  //tg.initGrid( 3, 3, 0.07, 0.08, origin);
  tg.initGrid( 3, 3, 0.01, 0.01, origin);
  tg.printGridLayout();
  tg.printGridMap();
  std::string trajPathStr = "ABCFEDGHIFEDA";
  //std::string trajPathStr = "AHCFIHDIEGA";
  ice_msgs::setTrajectory traj_msg_;
  traj_msg_.request.x =tg.str2Vec(trajPathStr);
  for(int i=0;i<traj_msg_.request.x.size();i++)
	  traj_msg_.request.t.push_back(2.0);

  // Set controller names
  std::vector<std::string> arm_controllers_default;
  std::vector<std::string> arm_controllers_new;
  arm_controllers_default.push_back("l_arm_controller");
  arm_controllers_new.push_back("pr2_adaptNeuroController");
  pr2manager.setControllers(arm_controllers_default, arm_controllers_new);

  ArmsCartesian arms;

  // ROS service clients
  ros::ServiceClient toggle_updateWeights_srv_ = nh.serviceClient<ice_msgs::setBool>("/pr2_adaptNeuroController/updateNNweights");
  ros::ServiceClient setNNWeights_srv_ = nh.serviceClient<ice_msgs::setNNweights>("/pr2_adaptNeuroController/setNNweights");
  ros::ServiceClient getNNWeights_srv_ = nh.serviceClient<ice_msgs::getNNweights>("/pr2_adaptNeuroController/getNNweights");
  ros::ServiceClient setTactileCalibration_srv_ = nh.serviceClient<ice_msgs::tactileCalibration>("/tactile/calibration");
  ros::ServiceClient publishExpData_srv_ = nh.serviceClient<std_srvs::Empty>("/pr2_adaptNeuroController/publishExpData");
  ros::ServiceClient captureData_srv_ = nh.serviceClient<std_srvs::Empty>("/pr2_adaptNeuroController/capture");
  ros::ServiceClient status_srv_ = nh.serviceClient<ice_msgs::setBool>("/tactile/status");
  ros::ServiceClient tactileFilterWeights_srv_ = nh.serviceClient<ice_msgs::tactileFilterWeights>("/tactile/filterWeights");
  ros::ServiceClient runExperiment_srv_ = nh.serviceClient<ice_msgs::setTrajectory>("/pr2_adaptNeuroController/runExperimentD");

  sound_play::SoundClient sc;

  // ROS messages
  ice_msgs::setBool setBool_msgs_;


  signal(SIGINT,quit);

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

    getKey(c);

    switch(c)
    {
      /******************************** Manger *******************************************/
    case '0':
    {
    	displayControllerMenu();
    	getKey(c);

		arm_controllers_default.clear();
		arm_controllers_new.clear();
		switch(c)
		{
		case '1':
		{
			arm_controllers_default.push_back("l_arm_controller");
			arm_controllers_new.push_back("pr2_adaptNeuroController");
			pr2manager.setControllers(arm_controllers_default, arm_controllers_new);
			break;
		}
		case '2':
		{
			arm_controllers_default.push_back("l_arm_controller");
			arm_controllers_new.push_back("l_cart");
			pr2manager.setControllers(arm_controllers_default, arm_controllers_new);
			break;
		}
		case '3':
		{
			arm_controllers_default.push_back("l_arm_controller");
			arm_controllers_default.push_back("r_arm_controller");
			arm_controllers_new.push_back("l_cart");
			arm_controllers_new.push_back("r_cart");
			pr2manager.setControllers(arm_controllers_default, arm_controllers_new);
			break;
		}
		default:
		{
			ROS_INFO_STREAM("No controller changed");
			break;
		}
		}
    	break;
    }
      case '1':
      {
    	  // controller on
    	  pr2manager.on(false);
    	  controller_active = true;
    	  break;
      }
      case '2':
      {
    	  // controller off
    	  pr2manager.off(false);
    	  controller_active = false;
    	  break;
      }
      case 'i':
      {
    	  // init robot
    	  pr2manager.robotInit(false);
    	  torsoHeight = 0.05;
    	  controller_active = false;
    	  break;
      }
      /******************************** Gripper  *****************************************/
      case 'o':
      {
    	  pr2manager.openGrippers(PR2Manager::LEFT);
    	  break;
      }
      case 'p':
      {
    	  pr2manager.closeGrippers(PR2Manager::LEFT);
    	  break;
      }
      case 'j':
      {
    	  pr2manager.openGrippers(PR2Manager::RIGHT);
    	  break;
      }
      case 'k':
      {
    	  pr2manager.closeGrippers(PR2Manager::RIGHT);
    	  break;
      }
      /******************************** Torso  *****************************************/
       case 'u':
       {
     	  if(torsoHeight<0.3)
     		  torsoHeight += deltaTorsoHeight;
     	  pr2manager.setTorso(torsoHeight);
     	  break;
       }
       case 'h':
       {
    	  if(torsoHeight>0.0)
    		  torsoHeight -= deltaTorsoHeight;
     	  pr2manager.setTorso(torsoHeight);
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

          getKey(c);

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
    	  typedef std::vector<geometry_msgs::Pose>::iterator it_type;

    	  std::cout<<"Executing trajectory: "<<trajPathStr<<"\n";

    	  ArmsCartesian::WhichArm arm = ArmsCartesian::LEFT;

    	  // AdaptNeuroController
    	  if (!runExperiment_srv_.call(traj_msg_))
    	  {
    		  ROS_ERROR("Failed to call runExperiment service!");
    	  }

    	  std::cout<<"Done!\n";
    	  break;
      }
      /******************************** calibration ****************************************/
      case 's':
      {
    	  bool calibrationRunning = false;

    	  double dx = 0.25;
    	  double dy = 0.25;

    	  int activeSensor = 2;
    	  int nextActiveSensor = 2;
    	  string dataFile = "default";
    	  string dataDir = "~/test_rtp";		// TODO get package path
    	  string topic = "/pr2_adaptNeuroController/experimentDataB";

    	  Eigen::VectorXd weights;
    	  Eigen::VectorXd avgWeights;
    	  weights.resize(8);
    	  avgWeights.resize(8);

    	  int curTrial = 0;
    	  int numTrials = 3;

           bool stop_menu2 = false;
           while(!stop_menu2)
           {

        	 displayCalibrationExperimentMenu(activeSensor, dataFile, curTrial, numTrials, calibrationRunning);

             // Get the next event from the keyboard
        	 if(!calibrationRunning)
        	 {
        		 getKey(c);

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
					 break;
				 }
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
					 break;
				 }
				 case 's':
				 {
					 calibrationRunning = true;
					 cout<<"Starting calibration!\n";
					 break;
				 }
				 case 'c':
				 {
					 // Capture data
					 cout<<"Start capturing data ...\n";
					 std_srvs::Empty empty_msgs;
					 if (!captureData_srv_.call(empty_msgs))
					 {
						 ROS_ERROR("Failed to call capture data service!");
					 }

					 // Check status
//					 cout<<"Waiting for all data to be recorded ... ";
//					 ice_msgs::setBool bool_msgs;
//					 bool_msgs.response.success = true;	// False when calibration is not longer response
//					 while(bool_msgs.response.success)
//					 {
//						 if(!status_srv_.call(bool_msgs))
//						 {
//							 ROS_ERROR("Failed to call status service!");
//						 }
//						 sleep(0.5);
//					 }
//					 cout<<"Recording complete!\n";
					 break;
				 }
				 case 'p':
				 {
					 // Publish data
					 cout<<"Publishing data ...\n";
					 std_srvs::Empty empty_msgs;
					 if (!publishExpData_srv_.call(empty_msgs))
					 {
						 ROS_ERROR("Failed to call publishing data service!");
					 }
					 break;
				 }
				 case 'f':
				 {
	            	 // Stop calibration
					 cout<<"Fixing weights ...\n";
					 ice_msgs::tactileCalibration tactileCalibration_msg;
					 tactileCalibration_msg.request.activeSensor = -1;
	            	 if (!setTactileCalibration_srv_.call(tactileCalibration_msg))
	            	 {
	            		 ROS_ERROR("Failed to call tactile calibration service!");
	            	 }
					 break;
				 }
				 case 'q':
				 {
					 stop_menu2 = true;
					 break;
				 }
				 case 'u':
				 {
					 weights.setZero();
					 avgWeights.setZero();

					 // Getting filter weights
					 ice_msgs::tactileFilterWeights fw_msg;
					 fw_msg.request.changeWeights = false;
					 for(int i=0;i<4;i++)
					 {
						 fw_msg.request.sensor = i;
						 cout<<"Getting filter weights from sensor "<<fw_msg.request.sensor<<" ...\n";
						 if (!tactileFilterWeights_srv_.call(fw_msg))
						 {
							 ROS_ERROR("Failed to call tactile/FilterWeights service!");
						 }

						 weights<<fw_msg.response.getWeights.f0,
								  fw_msg.response.getWeights.f1 ,
								  fw_msg.response.getWeights.f2 ,
								  fw_msg.response.getWeights.f3 ,
								  fw_msg.response.getWeights.f4 ,
								  fw_msg.response.getWeights.f5 ,
								  fw_msg.response.getWeights.f6 ,
								  fw_msg.response.getWeights.f7 ;
						 cout<<"Result: "<<weights.transpose()<<"\n---\n";

						 avgWeights += weights;
					 }

					 // Setting filter weights
					 avgWeights = avgWeights / 4;
					 fw_msg.request.changeWeights = true;
					 fw_msg.request.setWeights.f0 = avgWeights(0);
					 fw_msg.request.setWeights.f1 = avgWeights(1);
					 fw_msg.request.setWeights.f2 = avgWeights(2);
					 fw_msg.request.setWeights.f3 = avgWeights(3);
					 fw_msg.request.setWeights.f4 = avgWeights(4);
					 fw_msg.request.setWeights.f5 = avgWeights(5);
					 fw_msg.request.setWeights.f6 = avgWeights(6);
					 fw_msg.request.setWeights.f7 = avgWeights(7);
					 cout<<"Setting filter weights for all sensors ... \n";
					 if (!tactileFilterWeights_srv_.call(fw_msg))
					 {
						 ROS_ERROR("Failed to call tactile/FilterWeights service!");
					 }
					 cout<<"Result: "<<avgWeights.transpose()<<"\n---\n";
					 break;
				 }
				 default:
				 {
					 ROS_INFO_STREAM("Keycode not found: " << c);
					 break;
				 }
				 } //end switch

        	 }
        	 else	// Run calibration
             {
            	 tcsetattr(kfd, TCSANOW, &cooked);         // Use old terminal settings
            	 curTrial++;

            	 geometry_msgs::PoseStamped p;
            	 p.header.stamp = ros::Time::now();
            	 p.header.frame_id = "base_link";
            	 p.pose.orientation.w = 1;

            	 p.pose.position.x = 0.65;
				 p.pose.position.y = 0.35;
				 p.pose.position.z = 0.5;

            	 // Select sensor
            	 activeSensor = nextActiveSensor;
            	 ice_msgs::tactileCalibration tactileCalibration_msg;
     	 		 tactileCalibration_msg.request.start.position.x = 0.65;			// Green position
     	 		 tactileCalibration_msg.request.start.position.y = 0.35;
     	 		 tactileCalibration_msg.request.start.position.z = -0.125;
            	 tactileCalibration_msg.request.recordData = true;
            	 tactileCalibration_msg.request.activeSensor = activeSensor;
            	 switch(activeSensor)
            	 {
            	 	 case 2:	// Green -> blue (+x)
            	 		tactileCalibration_msg.request.distance = dx;
            	 		tactileCalibration_msg.request.time = 3;
            	 		nextActiveSensor = 3;	// Next sensor
            	 		break;
            	 	 case 3:	// Blue -> yellow (-y)
            	 		tactileCalibration_msg.request.start.position.x += dx;
            	 		tactileCalibration_msg.request.distance = dy;
            	 		tactileCalibration_msg.request.time = 3;
            	 		nextActiveSensor = 0;	// Next sensor
            	 		break;
            	 	 case 0:	// Yellow -> red (-x)
            	 		tactileCalibration_msg.request.start.position.x += dx;
            	 		tactileCalibration_msg.request.start.position.y -= dy;
            	 		tactileCalibration_msg.request.distance = dx;
            	 		tactileCalibration_msg.request.time = 3;
            	 		nextActiveSensor = 1;	// Next sensor
            	 		break;
            	 	 case 1:	// Red -> green (+y)
            	 		tactileCalibration_msg.request.start.position.y -= dy;
            	 		tactileCalibration_msg.request.distance = dy;
            	 		tactileCalibration_msg.request.time = 3;
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
            	 sc.say("Start!");

            	 // Save rtp file
            	 std::ostringstream convert;
            	 convert << curTrial;
            	 std::string cmd1 = string("rostopic echo -p ") + topic.c_str() + string(" > ") + dataDir.c_str() + string("/") + dataFile.c_str() + string("_") + convert.str() + string(".rtp &");
            	 cout<<"$ "<<cmd1.c_str()<<"\n";
            	 system( cmd1.c_str() );

            	 // Check status
            	 cout<<"Waiting for calibration run to complete ... \n";
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
            	 sc.say("Done!");
            	 cout<<" done!\n";

            	 // Publish data
            	 cout<<"Publishing data ... \n";
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
      /******************************** reference ****************************************/
      case 't':
      {
    	  typedef std::vector<geometry_msgs::Pose>::iterator it_type;
    	  ArmsCartesian::WhichArm arm = ArmsCartesian::LEFT;

    	  std::cout<<"Executing trajectory: "<<trajPathStr<<"\n";

    	  // Start capturing data

    	  // Ask user to follow pattern
    	  for(it_type iterator = traj_msg_.request.x.begin(); iterator != traj_msg_.request.x.end(); iterator++)
    	  {
    		  geometry_msgs::Pose pose = *iterator;

    		  //std::cout<<pose<<"\n";
    		  arms.moveToPose(arm,pose,"torso_lift_link",false);
    		  ros::Duration(3.0).sleep();
    	  }

    	  // Save results

    	  std::cout<<"Done!\n";

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

  return(0);
}

