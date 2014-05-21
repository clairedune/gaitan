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



#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <math.h>

#include <libgaitan/table.h>



using namespace std;

namespace gaitan
{
  
  
Table::Table():nbRow(10),nbCol(7){
  this->init();
}

Table::Table(int  rows, int  cols):nbRow(rows),nbCol(cols) {
  this->init();
}

void Table::init(){
  data.resize(this->nbRow,this->nbCol);
  for(int i=0; i< this->nbRow; i++)
    for(int j=0; j< this->nbCol; j++)
      data(i,j)=0;

}

void Table::resize(int rows, int cols){
  this->nbRow      = rows;
  this->nbCol      = cols;
  this->init();
}


//FIXE ME INVERSION !!
void Table::conservativeResize(int rows, int cols){
  this->nbRow      = rows;
  this->nbCol      = cols;
  data.conservativeResize(this->nbRow,this->nbCol);
}

void Table::print(){
 print(0, this->nbRow);
}


/*!
 * 
 *void print(int begin, int end)
 * \brief print a part of the data from the sample begin to the sample nuber end
 * 
 * 
 */ 
void Table::print(int deb, int end){

  if (deb<0) deb=0;
  else if (deb > this->nbRow)
  {   
    deb = this->nbRow;
    //std::cout << "changement deb : " << deb << std::endl;
  }
  if (end > this->nbRow) 
  {
    end = this->nbRow;
    //std::cout << "changement end : " << end << std::endl;

  }
  
  std::cout << std::endl<<"\t";
   for(int j=0;j<this->nbCol; j++){
        std::cout << "c"<< j << "\t";
   }
   
   for( int i=deb ; i < end ; i++ ){ 
     std::cout << endl <<"l"<< i << "\t";
     for(int j=0;j<this->nbCol; j++){
     std::cout 	<< std::setprecision(17) 
			<< this->data(i,j) 
			<< "\t";
    }
  } 
  
   std::cout <<std::endl;// << this->data << std::endl;
  
  
}




Table::~Table(){

}



int Table::load(string filename){
  ifstream file(filename.c_str(), ios::in);
  if(file)
    {
      //count lines
      int nbLine=0;
      string line;
      while(getline(file, line))
	    nbLine++;      
      
      this->nbRow = nbLine;
      this->init();
      
      // on revient au début du fichier
      file.clear();
      file.seekg(0,ios::beg);

      double x;
      for(int i=0;i<this->nbRow;i++){
	      for(int j=0;j<this->nbCol;j++){
          file >> x;
          this->data(i,j)=x;	 
	      }
      }
      file.close();
    }
  else 
    {
      cerr << "impossible d'ouvrir le fichier :: "<< filename<< endl;
      return -1;
    }
   
   return 1;
}

int Table::flush(std::vector<double> buffer, double initTime)
{
  
  std::cout << "buffer size" <<buffer.size() << std::endl;
  if (buffer.size()%4!=0){
    std::cerr << "The buffer does not contains enough values" << std::endl;
    //this->buffer.clear();
    return -1;
  }
  else{
    std::cout << "ici" << std::endl;
    double encTime0(initTime), encTime1(initTime);
   
    int nbData(4); // #0 encoder number #1 #2 #3 relative time
    int nbColumns(nbData+1); //#4 absolute time
    int nbSamples ((int)(buffer.size()/(nbData)));
    std::cout << "nbColumns : "<<nbColumns<< std::endl;
    std::cout << "nbSamples : "<<nbSamples<< std::endl;

    this->resize(nbSamples, nbColumns);
    std::cout << "nbCols : "<<this->getCols()<< std::endl;
    std::cout << "nbRows : "<<this->getRows()<< std::endl;

    
    int i=0;
    for(i=0 ; i<nbSamples ; i++ )
    {
      this->data(i,0) = buffer[i*nbData];
      this->data(i,1) = buffer[i*nbData+1];
      this->data(i,2) = buffer[i*nbData+2];
      this->data(i,3) = buffer[i*nbData+3];
      
      // compute the global time for each of the encoder
      if(buffer[i*nbColumns]==0){
          encTime0+=buffer[i*nbData+3]/1000000.0;
          this->data(i,4) = encTime0;
        }
      else if(buffer[i*nbData]==1){
          encTime1+=buffer[i*nbData+3]/1000000.0;
          this->data(i,4) = encTime1;
        }
    }
    std::cout << "last i " << i<<endl;

      std::cout << "Flushing values ok !!"<< std::endl;

    return 1;
 }
 
   std::cout << "Flushing values ok???"<< std::endl;

}

int Table::save(string filename, int precision){

	ofstream file(filename.c_str(), ios::out);

	if(file){
      		for(int i=0;i<this->nbRow;i++){		
			for(int j=0;j<this->nbCol;j++){
	  			file << setprecision(precision) << this->data(i,j) << "\t";	 
			}
			file << endl;	
      		}
      		file.close();
    	}
  else 
    {
      cerr << "impossible d'ouvrir le fichier"<<endl;
      return -1;
    }
 

return 1;
}




int Table::synchronize(Table &dataLeft,Table &dataRight ,Table & dataSynchro)
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
    //std::cout << "iterLeft<dataLeft.getRows() && iterRight<dataRight.getRows()" << std::endl;
		//if the two times are the same
		if(timeLeft==timeRight)
		{   
      //std::cout << "timeLeft==timeRight" << std::endl;
       
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
      //std::cout << "timeRight<timeLeft"<< std::endl;
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
