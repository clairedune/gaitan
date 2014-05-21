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

	Encoder * encoder = new Encoder(path);
  
  //--------------------------------------------//
  #if defined(VISP_HAVE_DISPLAY)
  // Create a window (700 by 700) at position (100, 200) with two graphics
  vpPlot A(2, 700, 700, 10, 10, "Odometry");
  // The first graphic contains 3 curves X,Y,theta 
  A.initGraph(0,3);
  // The second graphic contains 1 curve Y=f(X)
  A.initGraph(1,1);
  // The color of the curve in the first graphic is red
  A.setColor(0,0,vpColor::red);
  // The second curve on the first graph is green
  A.setColor(0,1,vpColor::green);
  // The third curve on the second graph is blue
  A.setColor(0,2,vpColor::blue);
  // The first curve of the second graph is blue
  A.setColor(1,0,vpColor::blue);

  int nbSamples(encoder->data->getRows());
  
  std::cout << "nb samples ::" << nbSamples << std :: endl;
  
  // remove the starting time to get a better view
  double timeT0(encoder->data->data(0,0));
  
  for (int i = 0; i < nbSamples; i++) 
  {
     float time  = encoder->data->data(i,0)-timeT0;
     
     if (time>0){
     //std::cout << "\n Time" << time << std::endl; 
     float x     = encoder->data->data(i,1);
     float y     = encoder->data->data(i,2);
     float theta = encoder->data->data(i,3);

    A.plot(0,0,time,x);
    A.plot(0,1,time,y);
    A.plot(0,2,time,theta);  
    A.plot(1,0,x,y);}
  }
  //return 0;
  #endif
  
  char a;
  std:: cin >> a;
  
  return 0;
}

