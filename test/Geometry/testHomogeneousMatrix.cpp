#include <sys/time.h> // to get time
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <math.h>

#include <Eigen/Dense>

#include <libgaitan/geometry.h>




using namespace gaitan;



void userInput (int argc, char** argv, double& x, double &y, double & theta)
{
  
  // get the path name
	if (argc>3){
		x = atof(argv[1]);
		y = atof(argv[2]);
		theta = atof(argv[3]);
	}
	else {
    x = 0.1;
    y = 0.0;
    theta = M_PI/12;
  }

std :: cout << "x: " << x << "\t y: "<< y << "\t theta: " << theta << std::endl;
  
}


// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{


 //------------- INIT -----------------------------------------//
  double x, y , theta;
  userInput(argc, argv, x, y, theta);
     
  //----- Create the transformation matrix ---- //
  Eigen::MatrixXf wMg(Eigen::MatrixXf::Identity(4,4));
  Geometry::homogeneousMatrix(x,y,theta, wMg);
  
  // ------ Display the transformation matrix ---- //
  
  std::cout << wMg << std::endl;
  
}


