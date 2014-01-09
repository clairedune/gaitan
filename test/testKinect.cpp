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
  Kinect* kinect = new Kinect();
  
  return 0;
}
