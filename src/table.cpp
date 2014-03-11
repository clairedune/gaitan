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
Table::Table():nbRow(10),nbCol(7){
  this->init();
}

Table::Table(int  cols, int  rows):nbRow(rows),nbCol(cols) {
  this->init();
}


void Table::init(){
   
  data.resize(this->nbRow, this->nbCol);
  
  //this->data = (double**)malloc(this->_nbRow*this->_nbCol*sizeof(double));
 
  //for(int i=0;i<this->_nbRow;i++){    
  //  data[i]=(double*)malloc(this->_nbCol*sizeof(double));
  //  for(int j=0;j<this->_nbCol; j++){
	//  data[i][j]=0;  
  //  }
  //}
  
}

void Table::resize(int cols, int rows){
  this->nbRow      = rows;
  this->nbCol      = cols;
  this->init();
}

void Table::print(){
 print(0, this->nbRow-1);
}

void Table::print(int deb, int end){
   for( int i=0 ; i < this->nbRow ; i++ ){ 
    if(i<=end && i>=deb){
     std::cout << endl << i << "\t";
     for(int j=0;j<this->nbCol; j++){
	  std::cout 	<< std::setprecision(11) 
			<< this->data(i,j) 
			<< "\t";
      }
    }
  }  
}




Table::~Table(){
//  for(int i=0;i<this->_nbRow;i++){
//    free(this->data[i]);
//  }
//  free(data);
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
      //FIXME : calculer le nombre de lignes à la volée this->_nbCol = 7;
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
      cerr << "impossible d'ouvrir le fichier"<<endl;
      return 0;
    }
   
   return 1;
}

int Table::save(string filename, int precision=10){

	ofstream file(filename.c_str(), ios::out);

	if(file){
      		for(int i=0;i<this->nbRow;i++){		
			for(int j=0;j<this->nbCol;j++){
	  			file <<  setprecision(precision) <<this->data(i,j) <<"\t";	 
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
