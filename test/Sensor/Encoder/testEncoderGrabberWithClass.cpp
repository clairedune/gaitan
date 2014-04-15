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
  std::string path,encoderFilename("encoder");
  
  if(argc>1)
      path = argv[1];
  else 
  {
    std::cerr <<"Usage error : "<< argv[0] << "+ data folder name." << std::endl;
    return 0;
  }
    
  std::cout << "Nom du chemin vers les données : ";
  std::cout << path << std::endl;
  std::cout << "Nom du fichier d'encoder : ";

  std::string filename = path+"/"+encoderFilename;
  
  
  //-----//
	Encoder * encoder = new Encoder();
  struct timeval now ;
  gettimeofday(&now,NULL);
	double initTime  = now.tv_sec + ((double)now.tv_usec)/1000000.0;
  //encoder->acquire();
 
 
  // Acquisition simulation //
  int lines (10);
  int columns (4);
 
  std::vector<double> values (columns*lines);
  for(int i=0;i<columns;i++)
   for(int j=0;j<lines;j++)
   {
     values[j*columns+i] = i+j*columns*100000;
     if(j%2==0) values[j*columns]=0;
     else values[j*columns]=1;
   }
   
  std::cout << "Buffer \n" <<std::endl; 
  for(int i=0; i<columns*lines; i++) 
  std::cout << values[i]<<std::endl; 
   
  encoder->buffer=values;
  
  //---------------------------//
  
  
  
  

  encoder->flush(initTime);
  encoder->print(0,4);


  
  std::cout << "Flushing values ok "<< std::endl;
  
   //compute odometry
  double L(0.529);
  encoder->odometry(L); 
  encoder->print(0,4);

  string outfile = path+"/odometry.dat"; 
  string outfileLeft = path+"/encoderL.dat"; 
  string outfileRight = path+"/encoderR.dat";
  string outfileSync = path+"/encoderS.dat";  

  encoder->save(outfile,17);
  encoder->saveLeft(outfileLeft);
  encoder->saveRight(outfileRight); 
  encoder->saveSynchro(outfileSync);  
  
  
  
  //--------------------------------------------//
  #if defined(VISP_HAVE_DISPLAY)
  // Create a window (700 by 700) at position (100, 200) with two graphics
  vpPlot A(2, 700, 700, 10, 10, "Odometry");
  // The first graphic contains 3 curves X,Y,theta 
  A.initGraph(0,4);
  A.initGraph(1,1);
  // The color of the curve in the first graphic is red
  A.setColor(0,0,vpColor::red);
  // The second curve is green
  A.setColor(0,1,vpColor::green);
  // The third curve is blue
  A.setColor(0,2,vpColor::blue);

  // The first curve of the second graph is blue
  A.setColor(1,0,vpColor::blue);

  int nbSamples(encoder->data->getRows());
  
  std::cout << "nb samples ::" << nbSamples << std :: endl;
  
  for (int i = 0; i < nbSamples; i++) 
  {
     float time = encoder->data->data(i,0);
     float x = encoder->data->data(i,0);
     float y = encoder->data->data(i,0);
     float theta = encoder->data->data(i,0);

    A.plot(0,0,time,x);
    A.plot(0,1,time,y);
    A.plot(0,2,time,theta);  
    A.plot(1,0,x,y);
  }
  //return 0;
  #endif
  
  char a;
  std:: cin >> a;
  
  return 0;
}

