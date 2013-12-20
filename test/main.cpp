#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <libgaitan/sensor.h>
#include <libgaitan/encoder.h>
#include <libgaitan/inertia.h>
#include <libgaitan/table.h>

using namespace std;
using namespace gaitan;

int main(int argc, char **argv) {
 

//argv[0] est le nom de la programme
  //argv[1] est le premier argument passé au programme
  // Get the data file name
  std::string path,
	      encoderFilename="encoder",
	      spatialFilename="spatial", 
	      groundTruthFilename="mocap";
  
  if(argc>1)
      path = argv[1];
  else 
  {
    std::cerr <<"Usage error : "<< argv[0] << " + data folder name." << std::endl;
		path="/home/dune/Documents/data/essai1";
  //  return 0;
  }
  
  std::cout << "Nom du chemin vers les données : ";
  std::cout << path <<std::endl;
  std::cout << "Nom du fichier d'encoder : ";
  std::cout << path+"/"+encoderFilename <<std::endl;
  std::cout << "Nom du fichier de la centrale : ";
  std::cout << path+"/"+spatialFilename <<std::endl;
  std::cout << "Nom du fichier de la mocap : ";
  std::cout << path+"/"+groundTruthFilename <<std::endl; 

  cout << endl<< "----- END----"<<endl;
  return 1;
}
