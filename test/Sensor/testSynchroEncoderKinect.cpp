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


//Fixe Me revoir les tableaux. 
int synchronize(Table &dataLeft, Table &dataRight ,Table & dataSynchro)
{
  //nbCol 
  int cols = dataLeft.getCols() + dataRight.getCols() - 1;
  
  //nbRow
  int rows = dataLeft.getRows()+ dataRight.getRows(); 
  //resize  output
  dataSynchro.resize(rows,cols);
 
  std::cout << "dataLeft.getRows() = " << dataLeft.getRows() <<" dataLeft.getCols() = " << dataLeft.getCols()<< std::endl ;
  std::cout << "dataRight.getRows() = " << dataRight.getRows() <<" dataRight.getCols() = " << dataRight.getCols()<< std::endl ;
  std::cout << "dataSynchro.getRows() = " << dataSynchro.getRows() <<" dataSynchro.getCols() = " << dataSynchro.getCols()<< std::endl ;

  int iterLeft(0);
  int iterRight(0);
  int iter(0);
  
  double timeLeft  = dataLeft.data(iterLeft,0);
  double timeRight = dataRight.data(iterRight,0);

  bool finished(false);
  while(!finished)
  {  
    
	if( iterLeft<dataLeft.getRows() && iterRight<dataRight.getRows())
	{
		//if the two times are the same
		if(timeLeft==timeRight)
		{   
			dataSynchro.data(iter,0) = timeLeft;
      
      for(int j=1 ; j<dataLeft.getCols() ; j++)
			  dataSynchro.data(iter,j) = dataLeft.data(iterLeft,j);
        
      for(int j=1 ; j<dataRight.getCols() ; j++)
			  dataSynchro.data(iter,dataLeft.getCols()+j-1) = dataRight.data(iterRight,j);
        
			iterLeft++;
			iterRight++;
      timeLeft = dataLeft.data(iterLeft,0);
      timeRight = dataRight.data(iterRight,0);
		}
		else if(timeLeft<timeRight)
		{

      dataSynchro.data(iter,0) = timeLeft;
      
      for(int j=1 ; j<dataLeft.getCols() ; j++)
			  dataSynchro.data(iter,j) = dataLeft.data(iterLeft,j);
        
      if (iter==0)
        for(int j=1 ; j<dataRight.getCols() ; j++)
          dataSynchro.data(iter,dataLeft.getCols()+j-1) = 0;
      else 
        for(int j=1 ; j<dataRight.getCols() ; j++)
          dataSynchro.data(iter,dataLeft.getCols()+j-1) = dataSynchro.data(iter-1,dataLeft.getCols()+j-1);
          
          
			iterLeft++;
			timeLeft = dataLeft.data(iterLeft,0);
		}
		else if (timeRight<timeLeft)
		{

      dataSynchro.data(iter,0) = timeRight;
      
      if(iter==0)
        for(int j=1 ; j<dataLeft.getCols() ; j++)
			    dataSynchro.data(iter,j) = 0;
      else  
        for(int j=1 ; j<dataLeft.getCols() ; j++)
			    dataSynchro.data(iter,j) = dataSynchro.data(iter-1,j);
        
        
      for(int j=1 ; j<dataRight.getCols() ; j++)
          dataSynchro.data(iter,dataLeft.getCols()+j-1) = dataRight.data(iterRight,j);
          
          
			iterRight++;
			timeRight = dataRight.data(iterRight,0);
		}	
    
	}  
           
	else if( iterLeft<dataLeft.getRows() && iterRight>=dataRight.getRows())
	{

       dataSynchro.data(iter,0) = timeLeft;
      
      for(int j=1 ; j<dataLeft.getCols() ; j++)
			  dataSynchro.data(iter,j) = dataLeft.data(iterLeft,j);
        
      if (iter==0)
        for(int j=1 ; j<dataRight.getCols() ; j++)
          dataSynchro.data(iter,dataLeft.getCols()+j-1) = 0;
      else 
        for(int j=1 ; j<dataRight.getCols() ; j++)
          dataSynchro.data(iter,dataLeft.getCols()+j-1) = dataSynchro.data(iter-1,dataLeft.getCols()+j-1);
          
          
			iterLeft++;
			timeLeft = dataLeft.data(iterLeft,0);
	}
	else  if( iterLeft>=dataLeft.getRows() && iterRight<dataRight.getRows())
	{

      dataSynchro.data(iter,0) = timeRight;
      
      if(iter==0)
        for(int j=1 ; j<dataLeft.getCols() ; j++)
			    dataSynchro.data(iter,j) = 0;
      else  
        for(int j=1 ; j<dataLeft.getCols() ; j++)
			    dataSynchro.data(iter,j) = dataSynchro.data(iter-1,j);
        
        
      for(int j=1 ; j<dataRight.getCols() ; j++)
          dataSynchro.data(iter,dataLeft.getCols()+j-1) = dataRight.data(iterRight,j);
          
          
			iterRight++;
			timeRight = dataRight.data(iterRight,0);
	}
	else 
	{
    finished = true; 
		cout << " ERREUR " << endl;
		cout << iter << " " <<  endl 
         << " iterL courant : " << iterLeft << " et le max " << dataLeft.getRows() 
         << " iterR courant : " << iterRight<< " et le max " << dataRight.getRows() << endl ;
	}	
	iter ++;
	if(iter>=dataSynchro.getRows()) finished = true; 
  }
  
  dataSynchro.conservativeResize(iter-1, dataSynchro.getCols());
  
  return 0;
  
}


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
  
  tableSmall.print();
  tableBig.print();
  
  // synchronize -------------------------------//
  Table synchro;
  synchronize(tableSmall, tableBig, synchro);
  synchro.print();
  
  
  return 1;
}
