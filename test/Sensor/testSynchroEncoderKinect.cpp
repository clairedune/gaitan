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
  
  Table tableSmall(10,2);
  Table tableBig(10,5);
  double timeStartSmall(1359736758);
  double timeStartBig(1359736763);
  
  // build test data ----------------------------//
  for (int i=0 ; i < tableSmall.getRows() ; i++)
    for (int j=0 ; j < tableSmall.getCols() ; j++)
      tableSmall.data(i,j) = j+i*tableSmall.getCols();
  
  for (int i=0 ; i < tableBig.getRows() ; i++)
    for (int j=0 ; j < tableBig.getCols() ; j++)
      tableBig.data(i,j) = 1000+(j+i*tableSmall.getCols());
      
  for (int i=0 ; i < tableSmall.getRows() ; i++)
      tableSmall.data(i,0) = timeStartSmall+3*i;
  
  for (int i=0 ; i < tableBig.getRows() ; i++)
      tableBig.data(i,0) = timeStartBig+2*i;
  
  std::cout << "nb de colonnes Small: " << tableSmall.getCols() << std::endl;
  std::cout << "nb de rang Small: "     << tableSmall.getRows() << std::endl;
  std::cout << "nb de colonnes Big: " << tableBig.getCols() << std::endl;
  std::cout << "nb de rang Big: "     << tableBig.getRows() << std::endl;
  
  tableSmall.print();
  tableBig.print();
  
  std::cout << std::endl << "begin synchro" << endl;
  
  // synchronize -------------------------------//
  Table synchro;
  Table::synchronize(tableSmall, tableBig, synchro);
  
  std::cout << "nb de colonnes : " << synchro.getCols() << std::endl;
  std::cout << "nb de rang : "     << synchro.getRows() << std::endl;
   
  synchro.print();
  
  
  return 1;
}
