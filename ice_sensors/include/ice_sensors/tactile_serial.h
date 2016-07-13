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
  int num_sensors;

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

TactileSerial(string port, unsigned long baud, int param_num_sensors)
{

num_sensors = param_num_sensors;
initFilter();

forceBias.resize(num_sensors);
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

	for(int i=0; i < num_sensors; i++)
	{
		digitalFilter* tmpPtr = new digitalFilter(1, true, b_lpfilt, a_lpfilt);
		sensorFilters.push_back(tmpPtr);
	}
}

bool getDataArrayFromSerialPort( Eigen::VectorXd & force  )
{

	force.resize(num_sensors);
	force.setZero();

//  std::cout<<"Result: "<<result<<"\n---\n";
    result = my_serial->readline(65536, "\n");

    std::vector<std::string> strvec;

    boost::algorithm::split(strvec,result,boost::algorithm::is_any_of(","), boost::algorithm::token_compress_on);

    // Display output (for debugging)
//    for( unsigned int i=0; i<strvec.size(); i++)
//    {
//    	std::cout<<"strvec["<<i<<"]: "<<strvec[i].c_str()<<"\n";
//    }

    // Check data: #,...,#,#,\n
    if(strvec.size() != num_sensors + 1)
    {
    	std::cout<<"Reading serial data failed (unexpected vector size: "<<strvec.size()<<")\n";
    	return false;
    }

    for( unsigned int i=0; i<num_sensors; i++)
    {
    	std::string str = strvec[i];

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
    	force(i) = boost::lexical_cast<double>(str);
//    	std::cout<<"force("<<i<<"): "<<force(i)<<"\n";
    }

//    if(firstRead)
//    {
//		forceBias = force;
//		firstRead = false;
//    }
//	force = force + Eigen::VectorXd::Ones(numSensors) - forceBias;

    // Filter values
    for( int i=0; i<num_sensors; i++)
    {
    	force(i) = sensorFilters[i]->getNextFilteredValue(force(i));
    	if(force(i) < 0)
    	{
    		force(i) = 0;
    	}
    }

    return true;
}

};
