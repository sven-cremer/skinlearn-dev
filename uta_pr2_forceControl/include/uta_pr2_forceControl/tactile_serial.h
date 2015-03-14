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

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;

class TactileSerial
{
  string result;
  serial::Serial *my_serial;

  bool firstRead;
  Eigen::VectorXd forceBias;

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

TactileSerial(string port_, unsigned long baud)
{

forceBias.resize(4);
firstRead=true;

  if( port_ == "-e" ) {
	  enumerate_ports();
  }

  // port, baudrate, timeout in milliseconds
  my_serial = new serial::Serial(port_, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial->isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;
}

TactileSerial(int argc, char **argv)
{

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
  my_serial = new serial::Serial(port, baud, serial::Timeout::simpleTimeout(1000));

  cout << "Is the serial port open?";
  if(my_serial->isOpen())
    cout << " Yes." << endl;
  else
    cout << " No." << endl;
}

~TactileSerial()
{}

bool getDataArrayFromSerialPort( Eigen::VectorXd & force  )
{
    result = my_serial->readline();
    std::vector<std::string> strvec;

    force << 1000,1000,0,0;
	
    boost::algorithm::split(strvec,result,boost::algorithm::is_any_of(","), boost::algorithm::token_compress_on);
	    
    for( unsigned int i=0; i<force.size(); i++)
    {
	force(i) = boost::lexical_cast<double>(strvec[i]);
    }

    if(firstRead)
    {
	forceBias = force;
	firstRead = false;
    }

	force = force + Eigen::VectorXd::Ones(force.size()) - forceBias;
    for( unsigned int i=0; i<force.size(); i++)
    {
	if(force(i) < 0){force(i) = 0;}
    }
}

};
