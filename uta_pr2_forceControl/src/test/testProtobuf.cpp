/*
 * testProtobuf.cpp
 *
 *  Created on: Apr 24, 2014
 *      Author: isura
 */

// See README.txt for information and build instructions.

#include <iostream>
#include <fstream>
#include <string>
#include "controllerFullData.pb.h"
using namespace std;

// This function fills in a Person message based on user input.
void PromptForAddress(dataPoint::Datum* datum)
{
	datum->set_dt( 0.05 );
}

// Main function:  Reads the entire address book from a file,
//   adds one person based on user input, then writes it back out to the same
//   file.
int main(int argc, char* argv[])
{
  // Verify that the version of the library that we linked against is
  // compatible with the version of the headers we compiled against.
  GOOGLE_PROTOBUF_VERIFY_VERSION;

  if (argc != 2) {
    cerr << "Usage:  " << argv[0] << " ADDRESS_BOOK_FILE" << endl;
    return -1;
  }

  dataPoint::controllerFullData controllerData;

  {
    // Read the existing address book.
    fstream input(argv[1], ios::in | ios::binary);
    if (!input) {
      cout << argv[1] << ": File not found.  Creating a new file." << endl;
    } else if (!controllerData.ParseFromIstream(&input))
    {
      cerr << "Failed to parse address book." << endl;
      return -1;
    }
  }

  // Add an address.
  PromptForAddress(controllerData.add_datum());

  {
    // Write the new address book back to disk.
    fstream output(argv[1], ios::out | ios::trunc | ios::binary);
    if (!controllerData.SerializeToOstream(&output))
    {
      cerr << "Failed to write address book." << endl;
      return -1;
    }
  }

  // Optional:  Delete all global objects allocated by libprotobuf.
  google::protobuf::ShutdownProtobufLibrary();

  return 0;
}
