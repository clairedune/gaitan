#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <eigen3/Eigen/Core>

#include <libgaitan/rgbdsensor.h>
#include <libgaitan/kinect.h>

using namespace std;
using namespace gaitan;
using namespace Eigen;

int main(int argc, char **argv) {
 
  Kinect * sensor = new Kinect();
  sensor->print();

  

  return 1;
}
