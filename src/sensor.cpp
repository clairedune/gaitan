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
#include <math.h>

#include <libgaitan/sensor.h>


//#define M_PI 3.1415


using namespace std;
namespace gaitan
{
Sensor::Sensor()
{
   this->_data        = new Table(4,10);
   this->initData();
}


Sensor::Sensor(string filename)
{ 
  if(this->readFile(filename)!=0)
   {
	 this->_data        = new Table(4,10);
   	this->initData();
   }
}

void Sensor::initData()
{
   this->_data->init();
}

void Sensor::print(int deb, int end){
  this->_data->print(deb, end);
}

void Sensor::print(){
  this->_data->print();
}

int Sensor::writeInFile(string outfile, int precision){
  this->_data->writeInFile(outfile,precision);
  return 0;
}

Sensor::~Sensor(){
 delete this->_data; 
}


int Sensor::readFile(string filename){
  return this->_data->createFromFile(filename);;
}



// First column is time
// Then the other columns are data
Table* Sensor::synchronizeZeroBloc(Sensor* s1, Sensor* s2)
{ 
  int nbRow1(s1->_data->_nbRow), nbCol1(s1->_data->_nbCol);  
  int nbRow2(s2->_data->_nbRow), nbCol2(s2->_data->_nbCol);
     
  // nb columns : (col_s1 - col_0_time) + (col_s2 - col_0_time) + col_time
  int nbCol = (nbCol1 - 1) + (nbCol2-1) + 1 ;
	
	// iterator init
  int iter1(0);
  int iter2(0);
  int iter(0);


  // get the first time in the two files
  double time1  = s1->_data->_data[iter1][0];
  double time2  = s2->_data->_data[iter2][0];
  // find the file that start after and shift the other iterator
  if (time1>time2){
	//cout << "Sensor 1 started after" << endl;
		do{
			iter2++;
			time2  = s2->_data->_data[iter2][0];
		}while (time1>time2 && iter2<s2->_data->_nbRow);
				
  }
  else {
	//cout << "Sensor 2 started after "<< endl;
		do{
			iter1++;
			time1  = s1->_data->_data[iter1][0];
		}while (time2>time1 && iter1<nbRow1);
 } 

 
 // max nb lines : raw_s1 + raw_s2
 int nbRow = nbRow1-iter1+ nbRow2-iter2; 
 
 // create result table
 Table * synchro = new Table(nbCol, nbRow);
 
 cout << "iter 1 : " << iter1 << " time 1 :  " << time1 << "iter 2 : " << iter2 << " time 2 " << time2 << endl; 
  
  bool finished(false);
  while(!finished)
  {  
   	if( iter1 < nbRow1 && iter2 < nbRow2){
		//if the two times are the same
		if(time1==time2){   
			synchro->_data[iter][0] = time1;
			for (int i=1;i<nbCol1;i++){
				synchro->_data[iter][i] = s1->_data->_data[iter1][i];
			}
			
			for (int i=nbCol1;i<nbCol;i++){
				synchro->_data[iter][i] = s2->_data->_data[iter2][i-nbCol1+1];
			}

			iter1++;
			iter2++;
			if (iter1<nbRow1)
  			time1       = s1->_data->_data[iter1][0];
			if (iter2<nbRow2)
  			time2       = s2->_data->_data[iter2][0];
		}
		else if(time1<time2)
		{

			synchro->_data[iter][0] = time1;

			for (int i=1;i<nbCol1;i++)
 			{
				synchro->_data[iter][i] = s1->_data->_data[iter1][i];
				
			}
			
			for (int i=nbCol1;i<nbCol;i++)
 			{
				if (iter2>0)
					synchro->_data[iter][i] = s2->_data->_data[iter2-1][i-nbCol1+1];
				else 
					synchro->_data[iter][i]=0;
			}

			iter1++;
			if (iter1<nbRow1)
  			time1       = s1->_data->_data[iter1][0];

		}
		else if (time2<time1)
		{

		         synchro->_data[iter][0] = time2;

                        for (int i=1;i<nbCol1;i++)
 			{
				if (iter1>0)
					synchro->_data[iter][i] = s1->_data->_data[iter1-1][i];
				else 
					synchro->_data[iter][i] = 0;
				
			}
			
			for (int i=nbCol1;i<nbCol;i++)
 			{
				synchro->_data[iter][i] = s2->_data->_data[iter2][i-nbCol1+1];
			}

			iter2++;
			if (iter2<nbRow2)
  			time2       = s2->_data->_data[iter2][0];
		}		
	}           
	else if( iter1<nbRow1 && iter2>=nbRow2)
	{  
		synchro->_data[iter][0] = time1;

                        for (int i=1;i<s1->_data->_nbCol;i++)
 			{
				synchro->_data[iter][i] = s1->_data->_data[iter1][i];
				
			}
			
			for (int i=nbCol1;i<nbCol;i++)
 			{
				if (iter2>0)
					synchro->_data[iter][i] = s2->_data->_data[iter2-1][i-nbCol1+1];
				else 
					synchro->_data[iter][i] = 0;
			}

			iter1++;
			if (iter1<nbRow1)
  			time1       = s1->_data->_data[iter1][0];
	}
	else  if( iter1>=nbRow1 && iter2<nbRow2)
	{
	         synchro->_data[iter][0] = time2;

                        for (int i=1;i<nbCol;i++)
 			{
				if (iter1>0)
					synchro->_data[iter][i] = s1->_data->_data[iter1-1][i];
				else 
					synchro->_data[iter][i] = 0;
				
			}
			
			for (int i=nbCol2;i<nbCol;i++)
 			{
				synchro->_data[iter][i] = s2->_data->_data[iter2][i-nbCol1+1];
			}

			iter2++;
			if (iter2<nbRow2)
  			time2       = s2->_data->_data[iter2][0];
	}
	else 
	{
		cout << " ERREUR" << endl;
		cout << iter << " " <<  endl 
                             << " iterL : " << iter1 << "et le max" << s1->_data->_nbRow 
		             << " iterR : " << iter2<< "et le max" << s2->_data->_nbRow << endl ;
	}	
	iter ++;
	if(iter>=nbRow) finished = true; 
  }
  
  return synchro;
  
}

}


