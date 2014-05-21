#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <eigen3/Eigen/Core>

#include <libgaitan/encoder.h>
#include <libgaitan/inertia.h>
#include <libgaitan/table.h>
#include <libgaitan/synchro.h>
#include <libgaitan/kinect.h>


using namespace std;
using namespace gaitan;

void userInput (int argc, char** argv, std::string& path)
{

 
  // get the path name
	if (argc>1){
		path = argv[1];
	}
	else {
    path ="/home/dune/Documents/data/encoder-kinect/essai2";
  }
  
  
}

int main(int argc, char **argv){
  

  //------------- Parameters-----------------------------------------//
  std::string path, pattern, fullPath;
  int nbIm, nbImBegin, nbImEnd;
  userInput(argc, argv, path);
  
  
  //------------ Create the kinect from file ------------------------//
  Kinect * kinect= new Kinect();  
  if(!kinect->loadConfFile(path))
  {
      std::cerr   <<
          "No configuration file found. You need to init the kinect first"  
          << std::endl;
      return -1;
  }
  
  if(!kinect->loadTimeSampling(path))
  {
      std::cerr   <<
        "No timeFile file found. "  
        << std::endl;
      return -2;
  }
  
  // -------------- Create the encoder from path --------------------//
  Encoder * encoder = new Encoder();
  if(!encoder->load(path))
  {
     std::cerr   <<
        "No odometry file found. You need to compute odometry first"  
        << std::endl;
    return -1;
  }
  
  //------------------ Synchro --------------------------------------//
  Table synchro;  
  Table::synchronize(*(kinect->data), *(encoder->data), synchro);
  synchro.print();
  
  std::string synchroFilename = path + "/synchro.dat" ;
  synchro.save("synchroFilename");
  
  
  return 1;
}
