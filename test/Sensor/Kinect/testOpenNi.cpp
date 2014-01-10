#include <iostream>
#include <fstream>
#include <string>
#include <OpenNI.h>

using namespace std;
  
int main(int argc, char **argv) {
  cout << endl<< "----- START ----"<<endl;

  openni::OpenNI::initialize();  
  callingopenni::OpenNI::getExtendedError();
  
  cout << endl<< "----- END ----"<<endl;
  return 1;
}
