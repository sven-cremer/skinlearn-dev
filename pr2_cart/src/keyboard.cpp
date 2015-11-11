/*
 * save_positions.cpp
 *
 *  Created on: Sep 28, 2013
 *      Author: sven
 */

//#include <thread>
#include <boost/thread.hpp>

#include <termios.h>
#include <signal.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <pr2_cart/pr2_cart_manager.h>
#include <ice_msgs/getState.h>


using namespace std;


static bool pr2_cartPull_active = false;			// TODO: get from PR2, this could be wrong OR send srv at beginning?
ice_msgs::getState currentState;

void displayMainMenu()
{
	string tmp1 = "Status: " + (string)(pr2_cartPull_active ?  "ON" : "OFF");
	puts("---------------------------");
	puts("MENU:   pr2_cartPull       ");
	puts(tmp1.c_str());
	puts("---------------------------");
	puts("Use '1' to turn pr2_cartPull ON");
	puts("Use '2' to turn pr2_cartPull OFF");
	puts("Use 'i' to initialize robot");
	puts("Use 'o' to open grippers");
	puts("Use 'c' to close grippers");
	puts(" ");
//	puts("Use 't' to set torque controller");
//	puts("Use 'v' to set velocity controller");
//	puts("Use 'i' to set impedance controller");
	puts("Use 's' to set gains");
	puts(" ");
	puts("Use 'q' to quit");
	puts(" ");
}

void displaySetMenu()
{
	puts("---------------------------");
	puts("### Torque controller ######");
	printf("1. restPGain \t = %.3f \n", currentState.response.restPGain);
	printf("2. restDGain \t = %.3f \n", currentState.response.restDGain);
	puts(" ");
	puts("### Velocity controller ####");
	printf("3. velPGain \t = %.3f \n", currentState.response.velPGain);
	printf("4. velDGain \t = %.3f \n", currentState.response.velDGain);
	printf("5. rotPGain \t = %.3f \n", currentState.response.rotPGain);
	printf("6. rotDGain \t = %.3f \n", currentState.response.rotDGain);
	puts(" ");
	puts("### Impedance controller ###");
	printf("7. Kp_vel \t = %.2f, %.2f, %.2f \n", currentState.response.Kp_vel_x,currentState.response.Kp_vel_y,currentState.response.Kp_vel_z);
	printf("8. Kd_vel \t = %.2f, %.2f, %.2f \n", currentState.response.Kd_vel_x,currentState.response.Kd_vel_y,currentState.response.Kd_vel_z);
	printf("9. Kp_vel \t = %.2f, %.2f, %.2f \n", currentState.response.Kp_rot_x,currentState.response.Kp_rot_y,currentState.response.Kp_rot_z);
	printf("0. Kd_vel \t = %.2f, %.2f, %.2f \n", currentState.response.Kd_rot_x,currentState.response.Kd_rot_y,currentState.response.Kd_rot_z);
	puts(" ");
	puts("### Thresholds ############");
	printf("r. rThresh \t = %.3f \n", currentState.response.rThresh);
	printf("p. psiThresh \t = %.3f \n", currentState.response.psiThresh);
	puts(" ");
	puts("Use 'i' to initialize gains");
	puts("Use 'q' to quit and return to main menu");
	puts(" ");
}

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


  ros::NodeHandle nh;

  PR2CartManager manager;

  manager.get_State_(&currentState);

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
    	  manager.on(false);
    	  pr2_cartPull_active = true;
    	  break;
      }
      case '2':
      {
    	  manager.off(false);
    	  pr2_cartPull_active = false;
    	  break;
      }
      case 'i':
      {
    	  manager.robotInit(false);
    	  pr2_cartPull_active = false;
    	  break;
      }
      /******************************** Gripper  *****************************************/
      case 'o':
      {
    	  manager.openGrippers();
    	  break;
      }
      case 'c':
      {
    	  manager.closeGrippers();
    	  break;
      }
      /******************************** Set gains ****************************************/
      case 's':
      {
        bool stop_setMenu = false;

        while(!stop_setMenu)
        {
          double value;
          double K_x, K_y, K_z;

          manager.get_State_(&currentState);
          displaySetMenu();

          // get the next event from the keyboard
          if(read(kfd, &c, 1) < 0)
          {
            perror("read():");
            exit(-1);
          }

          int nr = c - '0';										// Convert char to integer
          tcsetattr(kfd, TCSANOW, &cooked);         			// Use old terminal settings
          if ((nr >= 1 && nr <= 6)||(c == 'r')||(c == 'p'))
          {
        	  string tmp1 = (string)"*** Enter value for " + c + (string)" ***";
        	  puts(tmp1.c_str());
			  cin >> value;										// TODO: check user input
          }
          else if((nr >= 7 && nr <= 9)||(nr == 0))
          {
        	  string tmp1 = (string)"*** Enter Kx Ky Kz gains for " + c + (string)" ***";
        	  puts(tmp1.c_str());
			  cin >> K_x >> K_y >> K_z;
          }
          tcsetattr(kfd, TCSANOW, &raw);            			// Use new terminal settings

          switch(c)
          {
		  case 'r':
			  manager.set_rThresh_(value);
			  break;
		  case 'p':
			  manager.set_psiThresh_(value);
			  break;
		  case '1':
			  manager.set_restPGain_(value);
			  break;
		  case '2':
			  manager.set_restDGain_(value);
			  break;
		  case '3':
			  manager.set_velPGain_(value);
			  break;
		  case '4':
			  manager.set_velDGain_(value);
			  break;
		  case '5':
			  manager.set_rotPGain_(value);
			  break;
		  case '6':
			  manager.set_rotDGain_(value);
			  break;

		  case '7':
			  manager.set_Kp_vel_(K_x, K_y, K_z);
			  break;
		  case '8':
			  manager.set_Kd_vel_(K_x, K_y, K_z);
			  break;
		  case '9':
			  manager.set_Kp_rot_(K_x, K_y, K_z);
			  break;
		  case '0':
			  manager.set_Kd_rot_(K_x, K_y, K_z);
			  break;


				case 'i':
				  manager.initGains();
				  break;

				case 'q':
				  stop_setMenu = true;
				  break;
				default:
				   ROS_INFO_STREAM("Keycode not found: " << c);
				  break;
          } //end switch
        } // end while
        break;
      }

/*
 * NOTES:
 *
           	  // TODO: Set values in srv_setValue
        	  // TODO: call client
              //if (set_Kd_rot.call(srv)) {string temp = "-->Service response " + (string)(srv_setGains.response.completed ? "completed!" : "failed!"); puts(temp.c_str());}
              //else { ROS_ERROR("Failed to call service interface_server");}

        // TO ENTER TEXT:

        tcsetattr(kfd, TCSANOW, &cooked);         // Use old terminal settings

        char cmd[100];
        puts("*** Enter name of new motionLib ***");
        cin.getline(cmd,100);
        current_lib = string(cmd);
        srv.request.lib_name = current_lib;

        tcsetattr(kfd, TCSANOW, &raw);            // Use new terminal settings
*/
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

  manager.off();					// Turn off manager

  tcsetattr(kfd, TCSANOW, &cooked);	// Revert to old terminal settings

  ros::shutdown();
  spin_thread.join();
  return(0);
}

