// - Encoder simple -
// This example simply creates an Encoder handle, hooks the event handlers, and then waits for an encoder is attached.
// Once it is attached, the program will wait for user input so that we can see the event data on the screen when using the
// encoder.
//
// Copyright 2008 Phidgets Inc.  All rights reserved.
// This work is licensed under the Creative Commons Attribution 2.5 Canada License. 
// view a copy of this license, visit http://creativecommons.org/licenses/by/2.5/ca/

#include <stdio.h>
#include <phidget21.h>
#include <sys/time.h>

#include <visp/vpConfig.h>
#include <visp/vpPlot.h>



#include <libgaitan/encoder.h>


using namespace gaitan;
using namespace std;

int main(int argc, char **argv) {
  // argv[0] est le nom de la programme
  // argv[1] est le premier argument passé au programme
  // Get the data file name
  std::string path;
  
  if(argc>1)
      path = argv[1];
  else 
  {
    std::cerr <<"Usage error : "<< argv[0] << "+ data folder name." << std::endl;
    return 0;
  }
    
  std::cout << "Nom du chemin vers les données : ";
  std::cout << path << std::endl;
  
  
  
	Encoder * encoder = new Encoder();
  struct timeval now ;
  gettimeofday(&now,NULL);
	double initTime  = now.tv_sec + ((double)now.tv_usec)/1000000.0;
  encoder->acquire();
  encoder->flush(initTime);


  //compute odometry
  double L(0.529);
  encoder->odometry(L); 


  encoder->save(path);
  encoder->saveLeft(path);
  encoder->saveRight(path); 
  encoder->saveSynchro(path); 
  encoder->saveRaw(path); 
  
  
  
  
  
  return 0;
}

