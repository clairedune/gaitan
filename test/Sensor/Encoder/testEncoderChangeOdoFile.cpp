#include <visp/vpConfig.h>

#include <sys/time.h> // to get time
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>

#include <libgaitan/table.h>

using namespace gaitan;

int main(int argc, char ** argv) {
	
	std::string inputFileName,
				      outputFileName;

	if (argc>2){
		inputFileName = argv[1];
    outputFileName = argv[2];
	}
	else {
		std::cerr << "ERROR Usage : " << argv[0] << " input output" << std::endl;
		return -1;
	}
	
  // num absp relp absdist reldist relTime absTime
  Table * input = new Table(2,7) ;
  input->load(inputFileName);
  input -> print(0,10);
  Table * output = new Table(input->getRows(),5) ;
  
  for(int i = 0 ; i < input->getRows() ; i++)
   {
      output->data(i,0)= input->data(i,0);   
      output->data(i,1)= input->data(i,1);   
      output->data(i,2)= input->data(i,2);   
      output->data(i,3)= input->data(i,5);   
      output->data(i,4)= input->data(i,6);   
   }
  
  output->print(0,10);
  output->save(outputFileName);
  
  return 0;
}

