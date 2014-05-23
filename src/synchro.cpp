/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/


#include <libgaitan/synchro.h>


using namespace std;

namespace gaitan
{
 
 /*! Synchronize 2 timaTable using zero bloc*/
 int Synchro::synchronize(Table &dataLeft,Table &dataRight ,Table & dataSynchro)
{
  //nbCol 
  int cols = dataLeft.getCols() + dataRight.getCols() - 1;
  
  //nbRow
  int rows = dataLeft.getRows()+ dataRight.getRows(); 
  //resize  output
  dataSynchro.resize(rows,cols);
 

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
   // std::cout << "iterLeft<dataLeft.getRows() && iterRight<dataRight.getRows()" << std::endl;
		//if the two times are the same
		if(timeLeft==timeRight)
		{   
     // std::cout << "timeLeft==timeRight" << std::endl;
       
			dataSynchro.data(iter,0) = timeLeft;
      
      for(int j=1 ; j<dataLeft.getCols() ; j++)
			  dataSynchro.data(iter,j) = dataLeft.data(iterLeft,j);
        
      for(int j=1 ; j<dataRight.getCols() ; j++)
			  dataSynchro.data(iter,dataLeft.getCols()+j-1) = dataRight.data(iterRight,j);
        
			iterLeft++;
			iterRight++;
      if (iterLeft<dataLeft.getRows())timeLeft  = dataLeft.data(iterLeft,0);
      if (iterRight<dataRight.getRows())timeRight = dataRight.data(iterRight,0);
		}
		else if(timeLeft<timeRight)
		{
      //std::cout <<  "timeLeft<timeRight" << std::endl;
      
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
      if (iterLeft<dataLeft.getRows())
	      		timeLeft = dataLeft.data(iterLeft,0);
		}
		else if (timeRight<timeLeft)
		{
      std::cout << "timeRight<timeLeft"<< std::endl;
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
			if (iterRight<dataRight.getRows())
            timeRight = dataRight.data(iterRight,0);
		}	
    
	}  
           
	else if( iterLeft<dataLeft.getRows() && iterRight>=dataRight.getRows())
	{
       //std::cout << "iterLeft<dataLeft.getRows() && iterRight>=dataRight.getRows()" << std::endl;
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
			if (iterLeft<dataLeft.getRows())
              timeLeft = dataLeft.data(iterLeft,0);
	}
	else  if( iterLeft>=dataLeft.getRows() && iterRight<dataRight.getRows())
	{
      //std::cout << "iterLeft>=dataLeft.getRows() && iterRight<dataRight.getRows()" << std::endl;   
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
			if (iterRight<dataRight.getRows())timeRight = dataRight.data(iterRight,0);
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
  
  return 1;
  
}
 
}

