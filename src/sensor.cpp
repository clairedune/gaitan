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



using namespace std;
namespace gaitan
{
Sensor::Sensor()
{
   this->data        = new Table(4,10);
   this->initData();
}


Sensor::Sensor(string filename)
{ 
  if(this->load(filename)!=0)
   {
  	  this->data        = new Table(4,10);
    	this->initData();
   }
}

void Sensor::initData()
{
   this->data->init();
}

void Sensor::print(const int & deb, const int &end){
  this->data->print(deb, end);
}

void Sensor::print(){
  this->data->print();
}

int Sensor::save(const string&  outfile, const int &precision){
  this->data->save(outfile,precision);
  return 0;
}

Sensor::~Sensor(){
 delete this->data; 
}


int Sensor::load(const string &  filename){
  return this->data->load(filename);
}

//int Sensor::acquire(){}


// First column is time
// Then the other columns are data
Table* Sensor::synchronizeZeroBloc(Sensor* s1, Sensor* s2)
{ 
  int nbRow1(s1->data->getRows()), nbCol1(s1->data->getCols());  
  int nbRow2(s2->data->getRows()), nbCol2(s2->data->getCols());
     
  // nb columns : (col_s1 - col_0_time) + (col_s2 - col_0_time) + col_time
  int nbCol = (nbCol1 - 1) + (nbCol2-1) + 1 ;
	
	// iterator init
  int iter1(0);
  int iter2(0);
  int iter(0);


  // get the first time in the two files
  double time1  = s1->data->data(iter1,0);
  double time2  = s2->data->data(iter2,0);
  // find the file that start after and shift the other iterator
  if (time1>time2){
	//cout << "Sensor 1 started after" << endl;
		do{
			iter2++;
			time2  = s2->data->data(iter2,0);
		}while (time1>time2 && iter2<s2->data->getRows());
				
  }
  else {
	//cout << "Sensor 2 started after "<< endl;
		do{
			iter1++;
			time1  = s1->data->data(iter1,0);
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
			synchro->data(iter,0) = time1;
			for (int i=1;i<nbCol1;i++){
				synchro->data(iter,i)= s1->data->data(iter1,i);
			}
			
			for (int i=nbCol1;i<nbCol;i++){
				synchro->data(iter,i) = s2->data->data(iter2,i-nbCol1+1);
			}

			iter1++;
			iter2++;
			if (iter1<nbRow1)
  			time1       = s1->data->data(iter1,0);
			if (iter2<nbRow2)
  			time2       = s2->data->data(iter2,0);
		}
		else if(time1<time2)
		{

			synchro->data(iter,0) = time1;

			for (int i=1;i<nbCol1;i++)
 			{
				synchro->data(iter,i) = s1->data->data(iter1,i);
				
			}
			
			for (int i=nbCol1;i<nbCol;i++)
 			{
				if (iter2>0)
					synchro->data(iter,i) = s2->data->data(iter2-1,i-nbCol1+1);
				else 
					synchro->data(iter,i)=0;
			}

			iter1++;
			if (iter1<nbRow1)
  			time1       = s1->data->data(iter1,0);

		}
		else if (time2<time1)
		{

		         synchro->data(iter,0) = time2;

                        for (int i=1;i<nbCol1;i++)
 			{
				if (iter1>0)
					synchro->data(iter,i) = s1->data->data(iter1-1,i);
				else 
					synchro->data(iter,i) = 0;
				
			}
			
			for (int i=nbCol1;i<nbCol;i++)
 			{
				synchro->data(iter,i)= s2->data->data(iter2,i-nbCol1+1);
			}

			iter2++;
			if (iter2<nbRow2)
  			time2       = s2->data->data(iter2,0);
		}		
	}           
	else if( iter1<nbRow1 && iter2>=nbRow2)
	{  
		synchro->data(iter,0) = time1;

      for (int i=1;i<s1->data->getCols();i++)
 			{
				synchro->data(iter,i) = s1->data->data(iter1,i);
			}
			
			for (int i=nbCol1;i<nbCol;i++)
 			{
				if (iter2>0)
					synchro->data(iter,i) = s2->data->data(iter2-1,i-nbCol1+1);
				else 
					synchro->data(iter,i) = 0;
			}

			iter1++;
			if (iter1<nbRow1)
  			time1       = s1->data->data(iter1,0);
	}
	else  if( iter1>=nbRow1 && iter2<nbRow2)
	{
	         synchro->data(iter,0) = time2;

                        for (int i=1;i<nbCol;i++)
 			{
				if (iter1>0)
					synchro->data(iter,i) = s1->data->data(iter1-1,i);
				else 
					synchro->data(iter,i) = 0;
			}
			
			for (int i=nbCol2;i<nbCol;i++)
 			{
				synchro->data(iter,i) = s2->data->data(iter2,i-nbCol1+1);
			}

			iter2++;
			if (iter2<nbRow2)
  			time2       = s2->data->data(iter2,0);
	}
	else 
	{
		cout << " ERREUR" << endl;
		cout << iter << " " <<  endl 
         << " iterL : " << iter1 << "et le max" << s1->data->getRows() 
         << " iterR : " << iter2<< "et le max" << s2->data->getRows() << endl ;
	}	
	iter ++;
	if(iter>=nbRow) finished = true; 
  }
  
  return synchro;
  
}

}


