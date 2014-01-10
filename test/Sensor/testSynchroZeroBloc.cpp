#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <libgaitan/sensor.h>
#include <libgaitan/encoder.h>
#include <libgaitan/inertia.h>
#include <libgaitan/table.h>

#include <eigen3/Eigen/Core>

using namespace std;
using namespace gaitan;  
int main(int argc, char **argv) {
 

//argv[0] est le nom de la programme
  //argv[1] est le premier argument passé au programme
  // Get the data file name
  std::string path,
	      spatialFilename="spatial", 
	      spatialFilename2="spatial1";
  
  if(argc>1)
      path = argv[1];
  else 
  {
    std::cerr <<"Usage error : "<< argv[0] << " + data folder name." << std::endl;
    return 0;
  }
  
  std::cout << "Nom du chemin vers les données : ";
  std::cout << path <<std::endl;

  std:: string filename = path+"/"+spatialFilename;
  Inertia *inertia = new Inertia(filename);
  cout << endl << "---- 1st file ----"<< endl;
  inertia->Sensor::print(1,10);

  filename = path+"/"+spatialFilename2;
  Inertia *inertia2 = new Inertia(filename);
  cout << endl << "---- 2nd file ----"<< endl;
  inertia2->Sensor::print();

	cout << endl<<"Avant la fonction"<< endl;
	
  // on synchronise les données
  Table * data =  Sensor::synchronizeZeroBloc((Sensor*)inertia, (Sensor*)inertia2);
  cout << endl<< "----------- RESULTS------- "<<endl;
  //data->print();	
  cout << endl<< "----- END----"<<endl;
 //if(inertia) delete inertia;
//if(inertia) delete inertia2;
  return 1;
}
