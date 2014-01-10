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
    std::cerr <<"Usage error : "<< argv[0] << "+ data folder name." << std::endl;
    return 0;
  }
  
  std::cout << "Nom du chemin vers les données : ";
  std::cout << path <<std::endl;
  std::cout << "Nom du fichier d'encoder : ";
  std::cout << path+"/"+encoderFilename <<std::endl;
  std::cout << "Nom du fichier de la centrale : ";
  std::cout << path+"/"+spatialFilename <<std::endl;
  std::cout << "Nom du fichier de la mocap : ";
  std::cout << path+"/"+groundTruthFilename <<std::endl; 



  
  std::string filename = path+"/"+encoderFilename;
  Encoder *encoder = new Encoder(filename);   
  // test compute odometry
  double L(0.1);
  encoder->odometry(L); 
 encoder->print(1687,1697);
//  cout << endl<< "----- AVANT FICHIER----"<<endl;
//  string outfile = "test.dat"; 
//  encoder->writeInFile(outfile,17);

  filename = path+"/"+spatialFilename;
  Inertia *inertia = new Inertia(filename);
  inertia->print(0,10);
 // string out("testInertia.dat");
 // inertia->writeInFile(out, 17);

  // on synchronise les données
  Table * data =  Sensor::synchronizeZeroBloc((Sensor*)encoder, (Sensor*)inertia);
  data->print();	
	string fileout (path+"/encoder-spatial");
  data->writeInFile(fileout,16);
	
	cout << endl<< "----- END----"<<endl;
	
	
	delete inertia;
  delete encoder;
  return 1;
}
