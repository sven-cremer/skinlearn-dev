#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/split.hpp>
#include <boost/algorithm/string/trim.hpp>
#include <boost/lexical_cast.hpp>

#include <vector>
#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
	#include <windows.h>
#else
	#include <unistd.h>
#endif

#include "serial/serial.h"

#include <pr2_gripper_sensor_controller/digitalFilter.h>

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

class TactileSerial
{
  enum { numSensors = 4 };
  int num_sensors_per_patch;
  int num_sensors_per_mux;
  int num_muxes;
  int num_patches;
  int total_sensors;

  string result;
  serial::Serial *my_serial;

  bool firstRead;
  Eigen::VectorXd forceBias;

  std::vector<digitalFilter*> sensorFilters;

  float tempData;

private:
  void my_sleep(unsigned long milliseconds) {
  #ifdef _WIN32
        Sleep(milliseconds); // 100 ms
  #else
        usleep(milliseconds*1000); // 100 ms
  #endif
  }

  void enumerate_ports()
  {
  	  vector<serial::PortInfo> devices_found = serial::list_ports();
	  vector<serial::PortInfo>::iterator iter = devices_found.begin();

	  while( iter != devices_found.end() )
	  {
		  serial::PortInfo device = *iter++;
		  printf( "(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(), device.hardware_id.c_str() );
	  }
  }

void print_usage()
{
    cerr << "Usage: test_serial {-e|<serial port address>} ";
    cerr << "<baudrate> [test string]" << endl;
}

public:

TactileSerial(string port, unsigned long baud, int param_num_sensors, int param_num_patches)
{

num_sensors_per_patch = param_num_sensors;
num_patches = param_num_patches;
total_sensors = num_sensors_per_patch*num_patches;

num_muxes = 4;	// TODO make a variable
num_sensors_per_mux = (num_patches/num_muxes)*num_sensors_per_patch;	// 32

initFilter();

forceBias.resize(total_sensors);
firstRead=true;

  if( port == "-e" ) {
	  enumerate_ports();
  }

  // port, baudrate, timeout in milliseconds
  my_serial = new serial::Serial( port, baud, serial::Timeout::simpleTimeout(1000),
		  	  	  	  	  	  	  serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none );

  cout << "Is the serial port open?";
  if(my_serial->isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;
}

TactileSerial(int argc, char **argv)
{

initFilter();

forceBias.resize(4);
firstRead=true;

if(argc < 2)
  {
    print_usage();
  }

  // Argument 1 is the serial port or enumerate flag
  string port(argv[1]);

  if( port == "-e" ) {
	  enumerate_ports();
  }
  else if( argc < 3 ) {
	  print_usage();
  }

  // Argument 2 is the baudrate
  unsigned long baud = 0;
#if defined(WIN32) && !defined(__MINGW32__)
  sscanf_s(argv[2], "%lu", &baud);
#else
  sscanf(argv[2], "%lu", &baud);
#endif

  // port, baudrate, timeout in milliseconds
  my_serial = new serial::Serial( port, baud, serial::Timeout::simpleTimeout(1000),
		  	  	  	  	  	  	  serial::eightbits, serial::parity_none, serial::stopbits_one, serial::flowcontrol_none );

  cout << "Is the serial port open?";
  if(my_serial->isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;
}

~TactileSerial()
{
	my_serial->close();		// Close serial port
}

void initFilter()	// TODO read from parameter server
{
	// 1st order butterworth. low-pass 40 hz (fs 850)
	float b_lpfilt[] = {0.1296,  0.1296};
	float a_lpfilt[] = {1.0000, -0.7408};

	// 1st order butterworth. low-pass 100 hz
//	float b_lpfilt[] = {0.2452,  0.2452};
//	float a_lpfilt[] = {1.0000, -0.5095};

	// 1st order butterworth, fc=35Hz, fs=1000Hz
//	float b_lpfilt[] = {0.0994,  0.0994};
//	float a_lpfilt[] = {1.0000, -0.8012};

	for(int i=0; i < num_sensors_per_patch; i++)
	{
		digitalFilter* tmpPtr = new digitalFilter(1, true, b_lpfilt, a_lpfilt);
		sensorFilters.push_back(tmpPtr);
	}
}

double extractNumberFromString(std::string str)
{
	std::string tmp;
	std::size_t found = str.find_first_of("0123456789");
	while (found!=std::string::npos)
	{
		tmp += str[found];
		found =str.find_first_of("0123456789",found+1);
	}
	return boost::lexical_cast<double>(str);
}

bool getDataArrayFromSerialPort( Eigen::VectorXd & force, int & data_idx  )
{

	if(force.size() != total_sensors)
	{
		std::cerr<<"Reading serial data failed: force vector has unexpected length ("<<force.size()<<" instead of "<<total_sensors<<")\n";
		return false;
	}
	//force.setZero();

    result = my_serial->readline(65536, "\n");
//    std::cout<<"Result: "<<result<<"\n---\n";

    std::vector<std::string> strvec;

    boost::algorithm::split(strvec,result,boost::algorithm::is_any_of(","), boost::algorithm::token_compress_on);

    // Display output (for debugging)
    /*
    for( unsigned int i=0; i<strvec.size(); i++)
    {
    	std::cout<<"strvec["<<i<<"]: "<<strvec[i].c_str()<<"\n";
    }
    */

    // Check data: ID,#,...,#,#,\n
    if(strvec.size() != num_muxes + 2)
    {
    	std::cerr<<"Reading serial data failed: raw data has unexpected length ("<<strvec.size()<<" instead of "<<num_muxes + 2<<")\n";
    	return false;
    }

    // Extract sensor and patch index
    int patch_idx;
    int sensor_idx;
    std::string str = strvec[0];								// CX or CXX
    data_idx = (int)extractNumberFromString(str);

    // Check range
    if(data_idx < 0 || data_idx > num_sensors_per_mux-1)	// 0,...,31
    {
    	std::cerr<<"Reading serial data failed: extracted wrong sensor number ("<<sensor_idx<<")\n";
    	return false;
    }
    if(data_idx<num_sensors_per_patch)
    {
    	patch_idx = 0;
    	sensor_idx = data_idx;
    }
    else
    {
    	patch_idx = 1;
    	sensor_idx = data_idx - num_sensors_per_patch;
    }

    // Extract data
    for( unsigned int i=0; i<num_muxes; i++)
    {
    	str = strvec[i+1];

    	if (std::string::npos != str.find_first_not_of("0123456789"))
    	{
    		std::cout << "Found garbage in string! \n";

    		std::string tmp;
    		std::size_t found = str.find_first_of("0123456789");
    		while (found!=std::string::npos)
    		{
    			tmp += str[found];
    			found =str.find_first_of("0123456789",found+1);
    		}
    		str = tmp;
    	}
    	force(patch_idx*num_sensors_per_patch+sensor_idx) = boost::lexical_cast<double>(str);
    	//std::cout<<"force("<<patch_idx*num_sensors+i<<"): "<<force(patch_idx*num_sensors+i)<<"\n";
    	patch_idx +=2;
    }

//    if(firstRead)
//    {
//		forceBias = force;
//		firstRead = false;
//    }
//	force = force + Eigen::VectorXd::Ones(numSensors) - forceBias;

    // Filter values
    /*
    for( int i=0; i<num_sensors; i++)
    {
    	force(i) = sensorFilters[i]->getNextFilteredValue(force(i));
    	if(force(i) < 0)
    	{
    		force(i) = 0;
    	}
    }
	*/
    return true;
}

};
