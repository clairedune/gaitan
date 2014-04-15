#include <iostream>
#include <fstream>
#include <string>
#include <math.h>

#include <eigen3/Eigen/Core>

#include <libgaitan/table.h>


using namespace std;
using namespace gaitan;

int main(int argc, char **argv) {
   
  std::vector<double> values (28);
  for(int i=0;i<28;i++)
      values[i]=i;
   
  Table * table = new Table(2,5);
  
  table->flush(values);

  table->print();
  //delete encoder;
  return 1;
}
