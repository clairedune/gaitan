#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <eigen3/Eigen/Core>

#include <libgaitan/encoder.h>
#include <libgaitan/inertia.h>
#include <libgaitan/table.h>


using namespace std;
using namespace gaitan;

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
  Encoder *encoder = new Encoder(path);  

  encoder->print(0,10);

  //compute odometry
  double L(0.529);
  encoder->odometry(L); 
  encoder->print(5,10);

  encoder->save(path);
  encoder->saveLeft(path);
  encoder->saveRight(path); 
  encoder->saveSynchro(path); 
  encoder->saveRaw(path); 
  
  



  //delete encoder;
  return 1;
}
