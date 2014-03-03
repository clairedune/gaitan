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

#include <libgaitan/encoder.h>

//#define M_PI 3.1415


using namespace std;

namespace gaitan
{
Encoder::Encoder()
{


   this->_data        = new Table(4,10);
   this->_dataRaw     = new Table(7,10);  
   this->_dataLeft    = new Table(5,10);
   this->_dataRight   = new Table(5,10); 
   this->_dataSynchro = new Table(9,10);    

   this->initData();
}


Encoder::Encoder(string filename)
{


   this->_data        = new Table(4,10);
   this->_dataRaw     = new Table(7,10);  
   this->_dataLeft    = new Table(5,10);
   this->_dataRight   = new Table(5,10); 
   this->_dataSynchro = new Table(9,10);  

   this->initData();
  
   this->readFile(filename);

}

void Encoder::initData()
{
   this->_data->init(); 
   this->_dataRaw->init(); 
   this->_dataLeft->init();
   this->_dataRight->init();
   this->_dataSynchro->init();
  
}

int Encoder::acquire(const std::string &path , bool flagDisp)
{
   std::cout << "Encoder::acquire To be implemented ... " << std::endl;
   return 0;
    
  
  }



void Encoder::print(int deb, int end){
 cout <<endl<< "----RAW----" << endl;
  this->_dataRaw->print(deb, end);
  
  cout <<endl<<"----- LEFT -----" <<endl;
  this->_dataLeft->print(deb, end);
  
  cout <<endl<<"----- RIGHT -----" <<endl;
  this->_dataRight->print(deb, end);

  cout <<endl<<"----- SYNCHRO -----" <<endl;
  this->_dataSynchro->print(deb, end);

  cout <<endl<<"----- ODOMETRY -----" <<endl;
  this->_data->print(deb, end);
}

void Encoder::print(){
  cout <<endl<<"----- ODOMETRY -----" <<endl;
  this->_data->print();
}



Encoder::~Encoder(){
 delete this->_dataRight;
 delete this->_dataLeft;
 delete this->_dataSynchro;
 delete this->_dataRaw; 
 //delete this->_data; 
}



int Encoder::readFile(string filename)
{
  this->_dataRaw->createFromFile(filename);
  this->splitData();
  this->synchronizeData();
  return 0;
}

int Encoder::writeInFileLeft(string filename)
{
  return this->_dataLeft->writeInFile(filename,17);
 
}

int Encoder::writeInFileRight(string filename)
{
  return this->_dataRight->writeInFile(filename,17);
 
}


int Encoder::writeInFileSynchro(string filename)
{
  return this->_dataSynchro->writeInFile(filename,17);
}


/*!
 * 
 * Split right and left data from data read in a file
 * 
 * Warning : suppose that _data has been filled
 * #1:absTime #2:relPulse #3:relDist #4:absPulse #5:absDist
 *
 */
void Encoder:: splitData()
{ 

 cout << "Entrer dans la fonction split data"<< endl;
 
 this->_dataLeft->resize(5,this->_dataRaw->_nbRow);
 this->_dataRight->resize(5,this->_dataRaw->_nbRow);
 
 cout << "Les deux vecteurs sont crées"<< endl;
 
 double wheelSize(0.1);

 int indexLeft(0), indexRight(0); 

 // run on all the file lines
 for(int i=0;i<this->_dataRaw->_nbRow;i++)
  { 
    // encoder pulse value since the beginning of the experiment
    double absPulse ( this->_dataRaw->_data[i][1] );
    double absAngle ( absPulse/4*M_PI/180 );
    double absDist  ( absAngle/4*wheelSize );    

    // encoder relative value since the last sample
    double relPulse ( this->_dataRaw->_data[i][2] );
    double relAngle ( relPulse/4*M_PI/180 );
    double relDist  ( relAngle/4*0.1 );
  
    // time
    double absTime  ( this->_dataRaw->_data[i][6] ) ;
    
    // encoder number
    int num(this->_dataRaw->_data[i][0]);   

    // if the encoder is 0, then store data in left matrice else right
     if (num==0)
     { 
       this->_dataLeft->_data[indexLeft][0] = absTime;
       this->_dataLeft->_data[indexLeft][1] = relPulse;
       this->_dataLeft->_data[indexLeft][2] = relDist;
       this->_dataLeft->_data[indexLeft][3] = absPulse;
       this->_dataLeft->_data[indexLeft][4] = absDist;
       indexLeft++;
	
    }
    else
    {
       this->_dataRight->_data[indexRight][0] = absTime;
       this->_dataRight->_data[indexRight][1] = -relPulse;
       this->_dataRight->_data[indexRight][2] = -relDist;
       this->_dataRight->_data[indexRight][3] = -absPulse;
       this->_dataRight->_data[indexRight][4] = -absDist;
       indexRight++;
    }
   
   this->_dataLeft->_nbRow = indexLeft-1;
   this->_dataRight->_nbRow = indexRight-1;

  }
   
}


/*
* Build a folder with left and right data synchronised
* Input  _dataSynchro->_data : #0:absTime #1:relPulse  #2:relDist  #3:absPulse  #4:absDist
* Output _dataSynchro->_data : ZERO-BLOC
* #0:absTime 
* #1:relPulseL 
* #2:relDistL
* #3:absPulseL 
* #4:absDistL 
* #5:relPulseR 
* #6:relDistR
* #7:absPulseR 
* #8:absDistR
*/
int Encoder::synchronizeData()
{ 
  int nbCol(9);
  int nbRow(this->_dataRaw->_nbRow); 
  this->_dataSynchro->resize(nbCol, nbRow);

  int iterLeft(0);
  int iterRight(0);
  int iter(0);

  double timeLeft  = this->_dataLeft->_data[iterLeft][0];
  double timeRight = this->_dataRight->_data[iterRight][0];

  bool finished(false);
  while(!finished)
  {  
     /*cout << iter <<" : "<< _data->_nbRow << " | "
          << iter <<" : "<< _dataSynchro->_nbRow << " | "
          << iterLeft <<" : "<< _dataLeft->_nbRow<< " | "
          << iterRight <<" : "<< _dataRight->_nbRow<<  endl;*/
        // if the two iterators are smaller than _dataSynchro->_data size
	if( iterLeft<this->_dataLeft->_nbRow && iterRight<this->_dataRight->_nbRow)
	{
		//if the two times are the same
		if(timeLeft==timeRight)
		{   

			this->_dataSynchro->_data[iter][0] = timeLeft;

			this->_dataSynchro->_data[iter][1] = this->_dataLeft->_data[iterLeft][1];
			this->_dataSynchro->_data[iter][2] = this->_dataLeft->_data[iterLeft][2];
			this->_dataSynchro->_data[iter][3] = this->_dataLeft->_data[iterLeft][3];
			this->_dataSynchro->_data[iter][4] = this->_dataLeft->_data[iterLeft][4];


			this->_dataSynchro->_data[iter][5] = this->_dataRight->_data[iterRight][1];
			this->_dataSynchro->_data[iter][6] = this->_dataRight->_data[iterRight][2];
			this->_dataSynchro->_data[iter][7] = this->_dataRight->_data[iterRight][3];
			this->_dataSynchro->_data[iter][8] = this->_dataRight->_data[iterRight][4];


			iterLeft++;
			iterRight++;
  			timeLeft       = this->_dataLeft->_data[iterLeft][0];
  			timeRight      = this->_dataRight->_data[iterRight][0];
		}
		else if(timeLeft<timeRight)
		{

                     this->_dataSynchro->_data[iter][0] = timeLeft;

			this->_dataSynchro->_data[iter][1] = this->_dataLeft->_data[iterLeft][1];
			this->_dataSynchro->_data[iter][2] = this->_dataLeft->_data[iterLeft][2];
			this->_dataSynchro->_data[iter][3] = this->_dataLeft->_data[iterLeft][3];
			this->_dataSynchro->_data[iter][4] = this->_dataLeft->_data[iterLeft][4];


			this->_dataSynchro->_data[iter][5] = 0;
			this->_dataSynchro->_data[iter][6] = 0;
		        if(iterRight>0){
				this->_dataSynchro->_data[iter][7] = this->_dataRight->_data[iterRight-1][3];
		        	this->_dataSynchro->_data[iter][8] = this->_dataRight->_data[iterRight-1][4];
			}
			iterLeft++;
  			timeLeft   = this->_dataLeft->_data[iterLeft][0];
		}
		else if (timeRight<timeLeft)
		{

		        this->_dataSynchro->_data[iter][0] = timeRight;

			this->_dataSynchro->_data[iter][1] = 0;
			this->_dataSynchro->_data[iter][2] = 0;
			if(iterLeft>0){
				this->_dataSynchro->_data[iter][3] = this->_dataLeft->_data[iterLeft-1][3];
				this->_dataSynchro->_data[iter][4] = this->_dataLeft->_data[iterLeft-1][4];
			}

			this->_dataSynchro->_data[iter][5] = this->_dataRight->_data[iterRight][1];
			this->_dataSynchro->_data[iter][6] = this->_dataRight->_data[iterRight][2];
			this->_dataSynchro->_data[iter][7] = this->_dataRight->_data[iterRight][3];
			this->_dataSynchro->_data[iter][8] = this->_dataRight->_data[iterRight][4];

			iterRight++;
  			timeRight      = this->_dataRight->_data[iterRight][0];
		}		
	}           
	else if( iterLeft<=this->_dataLeft->_nbRow && iterRight>=this->_dataRight->_nbRow)
	{

		        this->_dataSynchro->_data[iter][0] = timeLeft;

			this->_dataSynchro->_data[iter][1] = this->_dataLeft->_data[iterLeft][1];
			this->_dataSynchro->_data[iter][2] = this->_dataLeft->_data[iterLeft][2];
			this->_dataSynchro->_data[iter][3] = this->_dataLeft->_data[iterLeft][3];
			this->_dataSynchro->_data[iter][4] = this->_dataLeft->_data[iterLeft][4];


			this->_dataSynchro->_data[iter][5] = 0;
			this->_dataSynchro->_data[iter][6] = 0;
		        if(iterRight>0){
				this->_dataSynchro->_data[iter][7] = this->_dataRight->_data[iterRight-1][3];
		        	this->_dataSynchro->_data[iter][8] = this->_dataRight->_data[iterRight-1][4];
			}
		iterLeft++;
  		timeLeft       = this->_dataLeft->_data[iterLeft][0];
	}
	else  if( iterLeft>=this->_dataLeft->_nbRow && iterRight<=this->_dataRight->_nbRow)
	{

		        this->_dataSynchro->_data[iter][0] = timeRight;

			this->_dataSynchro->_data[iter][1] = 0;
			this->_dataSynchro->_data[iter][2] = 0;
			if(iterLeft>0){
				this->_dataSynchro->_data[iter][3] = this->_dataLeft->_data[iterLeft-1][3];
				this->_dataSynchro->_data[iter][4] = this->_dataLeft->_data[iterLeft-1][4];
			}


			this->_dataSynchro->_data[iter][5] = this->_dataRight->_data[iterRight][1];
			this->_dataSynchro->_data[iter][6] = this->_dataRight->_data[iterRight][2];
			this->_dataSynchro->_data[iter][7] = this->_dataRight->_data[iterRight][3];
			this->_dataSynchro->_data[iter][8] = this->_dataRight->_data[iterRight][4];
		iterRight++;
  		timeRight      = this->_dataRight->_data[iterRight][0];
	}
	else 
	{
		cout << " ERREUR" << endl;
		cout << iter << " " <<  endl 
                             << " iterL : " << iterLeft << "et le max" << this->_dataLeft->_nbRow 
		             << " iterR : " << iterRight<< "et le max" << this->_dataRight->_nbRow << endl ;
	}	
	iter ++;
	if(iter>=this->_dataSynchro->_nbRow) finished = true; 
  }
  
  return 0;
  
}

//function [x,y,theta] = computeWalkerPosition(time, delta0, delta1, L)
/*
 * _dataSynchro->_data :
* #0:absTime 
* #1:relPulseL 
* #2:relDistL
* #3:absPulseL 
* #4:absDistL 
* #5:relPulseR 
* #6:relDistR
* #7:absPulseR 
* #8:absDistR
 * L : distance between the two wheelSize
 */  
int Encoder::odometry(double L)
{
       if(L<=0) return -1;
       this->_data->resize(4, this->_dataSynchro->_nbRow);

       double dOmega(0), dS(0), deltaRight(0), deltaLeft(0);
       double xprec(0), yprec(0), thetaprec(0),x(0),y(0),theta(0) ;
  
       for(int i=0; i< this->_dataSynchro->_nbRow;i++)
        {
           deltaRight = this->_dataSynchro->_data[i][6] ;
           deltaLeft  = this->_dataSynchro->_data[i][2] ;
           dOmega     = (deltaRight-deltaLeft) / (L) ;
           dS         = (deltaRight+deltaLeft)/2;

           if  (dOmega!=0){
	           x  = xprec + sin(M_PI*dOmega/2)/(M_PI*dOmega/2)*(dS*cos(thetaprec+dOmega/2));
	           y  = yprec + sin(M_PI*dOmega/2)/(M_PI*dOmega/2)*(dS*sin(thetaprec+dOmega/2));      
	   }
           else {
	           x  = xprec +(dS*cos(thetaprec+dOmega/2));
	           y  = yprec +(dS*sin(thetaprec+dOmega/2));      
	   }

           theta      = thetaprec + dOmega;
           
           // store the results
	   this->_data->_data[i][0] =  this->_dataSynchro->_data[i][0];
           this->_data->_data[i][1] = x;
	   this->_data->_data[i][2] = y;	   	
	   this->_data->_data[i][3] = theta;
	   
           // update prec
           xprec = x ;
   	   yprec = y ;
	   thetaprec = theta ; 		           		
        }
  
     return  0;

}
}

