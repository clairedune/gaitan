// ce code permet 
// 1. d'acquerir des images grace à une kinect
// On suppose qu'une seule kinect est branchée.

#include <libgaitan/kinect.h>

#include <iostream>
#include <fstream>
#include <string>

using namespace std;
using namespace gaitan;

int main(int argc, char**argv)
{ 
  std::string path, option;

  if (argc<2){
		option = "-d";
    std::cerr << "ERROR Usage : " << argv[0] << " -d for display -a for acquire pathname " << std::endl;

	}
	else if (argc>2){
		path = argv[2];
    option = argv[1];
	}
	else {
		std::cerr << "ERROR Usage : " << argv[0] << " -d for display -a for acquire pathname " << std::endl;
		return -1;
	}
  
  // creation of the sensor
  Kinect* kinect = new Kinect();
  
  /*switch(option){
    case "-d":
      kinect->display();
      break;
    case "-a":
      kinect->acquire(path, true);
      break;
    default:
      std::cout << "unrecognize option" << std::endl;
      break;  
    )*/
  if(option == "-d")
      kinect->display();
  else if (option=="-a")
      kinect->acquire(path, true);
  else 
      std::cout << "unrecognize option" << std::endl;
      
    
  return 0;
}
