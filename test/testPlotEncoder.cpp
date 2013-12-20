#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <eigen3/Eigen/Core>

#include <libgaitan/plot.h>
#include <libgaitan/encoder.h>
#include <libgaitan/inertia.h>
#include <libgaitan/table.h>


using namespace std;
using namespace gaitan;

int main(int argc, char **argv) {


  std::string filename = "/home/dune/Documents/data/essai1/encoder";
  Encoder *encoder = new Encoder(filename);  
  cout << "creation " <<endl;
  // test compute odometry
  double L(0.529);
  encoder->odometry(L); 

  Plot::plotEncoder(encoder);
  

  return 1;
}
