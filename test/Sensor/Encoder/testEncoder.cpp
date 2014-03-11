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
  Encoder *encoder = new Encoder(filename);  

  //compute odometry
  double L(0.529);
  encoder->odometry(L); 
  encoder->print(5,10);


  string outfile = path+"/odometry.dat"; 
  string outfileLeft = path+"/encoderL.dat"; 
  string outfileRight = path+"/encoderR.dat";
  string outfileSync = path+"/encoderS.dat";  

  encoder->save(outfile,17);
  encoder->saveLeft(outfileLeft);
  encoder->saveRight(outfileRight); 
  encoder->saveSynchro(outfileSync);

  //delete encoder;
  return 1;
}
