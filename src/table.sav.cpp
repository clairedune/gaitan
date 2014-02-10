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
#include <iomanip>
#include <math.h>

#include <libgaitan/table.h>



using namespace std;

namespace gaitan
{
Table::Table(){
  this->_nbRow      = 10;
  this->_nbCol      = 7;
  this->init();
}

Table::Table(int  col, int  raw){
  this->_nbRow      = raw;
  this->_nbCol      = col;
  this->init();
}


void Table::init(){
  this->_data = (double**)malloc(this->_nbRow*this->_nbCol*sizeof(double));
 
  for(int i=0;i<this->_nbRow;i++){    
    _data[i]=(double*)malloc(this->_nbCol*sizeof(double));
    for(int j=0;j<this->_nbCol; j++){
	  _data[i][j]=0;  
    }
  }
  
}

void Table::resize(int col, int raw){
  this->_nbRow      = raw;
  this->_nbCol      = col;
  this->init();
  
}

void Table::print(){
 print(0, this->_nbRow-1);
}

void Table::print(int deb, int end){
   for( int i=0 ; i < this->_nbRow ; i++ ){ 
    if(i<=end && i>=deb){
     std::cout << endl << i << "\t";
     for(int j=0;j<this->_nbCol; j++){
	  std::cout 	<< std::setprecision(11) 
			<< this->_data[i][j] 
			<< "\t";
      }
    }
  }  
}




Table::~Table(){
  for(int i=0;i<this->_nbRow;i++){
    free(this->_data[i]);
  }
  free(_data);
}



int Table::createFromFile(string filename){
  ifstream file(filename.c_str(), ios::in);
  if(file)
    {
      //count lines
      int nbLine=0;
      string line;
      while(getline(file, line))
	nbLine++;      
      
      this->_nbRow = nbLine;
      //FIXME : calculer le nombre de lignes à la volée this->_nbCol = 7;
      this->init();
      
      // on revient au début du fichier
      file.clear();
      file.seekg(0,ios::beg);

      double x;
      for(int i=0;i<this->_nbRow;i++){
	for(int j=0;j<this->_nbCol;j++){
	  file >>  this->_data[i][j];	 
	}
      }
      file.close();
    }
  else 
    {
      cerr << "impossible d'ouvrir le fichier"<<endl;
      return -1;
    }
   
   return 0;
}

int Table::writeInFile(string filename, int precision=10){

	ofstream file(filename.c_str(), ios::out);

	if(file){
      		for(int i=0;i<this->_nbRow;i++){		
			for(int j=0;j<this->_nbCol;j++){
	  			file <<  setprecision(precision) <<this->_data[i][j] <<"\t";	 
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
 

return 0;
}
}
