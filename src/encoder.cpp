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
#include <sys/time.h>
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>
#include <phidget21.h>
#include <libgaitan/encoder.h>
#include <libgaitan/rgbdsensor.h>

//#define M_PI 3.1415

using namespace std;

namespace gaitan
{
  Encoder::Encoder():
  dataRawFilename("encoder.dat"),
  dataLeftFilename("encoderL.dat"),
  dataRightFilename("encoderR.dat"),
  dataSynchroFilename("encoderSync.dat"),
  dataOdometryFilename("encoderOdo.dat")
  {//:encoderHandle(0){
 //  CPhidgetEncoder_create(&this->encoderHandle);
   
   this->data        = new Table(10,4);
   this->dataRaw     = new Table(10,5);  
   this->dataLeft    = new Table(10,5);
   this->dataRight   = new Table(10,5); 
   this->dataSynchro = new Table(10,9);    
   this->initData();
   
   
   
}


Encoder::Encoder(string path):
dataRawFilename("encoder.dat"),
dataLeftFilename("encoderL.dat"),
dataRightFilename("encoderR.dat"),
dataSynchroFilename("encoderSync.dat"),
dataOdometryFilename("encoderOdo.dat")
{
   this->data        = new Table(10,4);
   this->dataRaw     = new Table(10,5);  
   this->dataLeft    = new Table(10,5);
   this->dataRight   = new Table(10,5); 
   this->dataSynchro = new Table(10,9);  
   this->initData();
   
   this->load(path);
}

void Encoder::initData()
{
   this->data->init(); 
   this->dataRaw->init(); 
   this->dataLeft->init();
   this->dataRight->init();
   this->dataSynchro->init();
  
}

/*! Flush the buffer in the data 
 * clear the dataRaw buffer and fill it again 
 */
 
 int Encoder::flush( double initTime)
{
  std::cout << "buffer size" <<buffer.size() << std::endl;
  if (buffer.size()%4!=0){
    std::cerr << "The buffer does not contains enough values" << std::endl;
    //this->buffer.clear();
    return -1;
  }
  else{
    
    double encTime0(initTime), encTime1(initTime);
   
    int nbData(4); // #0 encoder number #1 #2 #3 relative time
    int nbColumns(nbData+1); //#4 absolute time
    int nbSamples ((int)(buffer.size()/(nbData)));
    this->dataRaw->resize(nbSamples, nbColumns);

    
    int i=0;
    for(i=0 ; i<nbSamples ; i++ )
    {
      this->dataRaw->data(i,0) = buffer[i*nbData];
      this->dataRaw->data(i,1) = buffer[i*nbData+1];
      this->dataRaw->data(i,2) = buffer[i*nbData+2];
      this->dataRaw->data(i,3) = buffer[i*nbData+3];
      
      if(buffer[i*nbData]==0){
          encTime0+=buffer[i*nbData+3]/1000000.0;
          this->dataRaw->data(i,4) = encTime0;
        }
      else{
          encTime1+=buffer[i*nbData+3]/1000000.0;
          this->dataRaw->data(i,4) = encTime1;
        }
    }
    // once the raw data is filled, split the data
    this->splitData();
    this->synchronizeData();
    return 1;
 }
}


 



int Encoder::acquire(const std::string &path , bool flagDisp)
{
  this->acquire(); // fill in the buffer 
  return 0;
}


int Encoder::acquire()
{
  int result;
	const char *err;

	//Declare an encoder handle
	CPhidgetEncoderHandle encoder = 0;

	//create the encoder object
	CPhidgetEncoder_create(&encoder);
  
	//Set the handlers to be run when the device is plugged in or opened from software, unplugged or closed from software, or generates an error.
	CPhidget_set_OnAttach_Handler((CPhidgetHandle)encoder, this->attachHandler, NULL);
	CPhidget_set_OnDetach_Handler((CPhidgetHandle)encoder, detachHandler, NULL);
	CPhidget_set_OnError_Handler((CPhidgetHandle)encoder, errorHandler, NULL);

	//Registers a callback that will run if an input changes.
	//Requires the handle for the Phidget, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetEncoder_set_OnInputChange_Handler(encoder, inputChangeHandler, NULL);

	//Registers a callback that will run if the encoder changes.
	//Requires the handle for the Encoder, the function that will be called, and an arbitrary pointer that will be supplied to the callback function (may be NULL).
	CPhidgetEncoder_set_OnPositionChange_Handler (encoder, positionChangeHandler, &this->buffer);

	CPhidget_open((CPhidgetHandle)encoder, -1);

	//get the program to wait for an encoder device to be attached
	printf("Waiting for encoder to be attached....");
	if((result = CPhidget_waitForAttachment((CPhidgetHandle)encoder, 10000)))
	{
		CPhidget_getErrorDescription(result, &err);
		printf("Problem waiting for attachment: %s\n", err);
		return 0;
	}

	//Display the properties of the attached encoder device
	displayProperties(encoder);

	//read encoder event data
	printf("Reading.....\n");

	//keep displaying encoder data until user input is read
	printf("Press any key to end\n");
	getchar();

	//since user input has been read, this is a signal to terminate the program so we will close the phidget and delete the object we created
	printf("Closing...\n");
	CPhidget_close((CPhidgetHandle)encoder);
	CPhidget_delete((CPhidgetHandle)encoder);

	//all done, exit
	return 0;
}


void Encoder::print(int deb, int end){
  
  cout <<endl<< "----RAW----" << endl;
  this->dataRaw->print(deb, end);
  
  cout <<endl<<"----- LEFT -----" <<endl;
  this->dataLeft->print(deb, end);
  
  cout <<endl<<"----- RIGHT -----" <<endl;
  this->dataRight->print(deb, end);

  cout <<endl<<"----- SYNCHRO -----" <<endl;
  this->dataSynchro->print(deb, end);

  cout <<endl<<"----- ODOMETRY -----" <<endl;
  this->data->print(deb, end);
}



void Encoder::print(){
  cout <<endl<<"----- BUFFER -----" <<endl;
 // this->buffer->print();
  
  for( std::vector<double>::const_iterator i = this->buffer.begin(); i != this->buffer.end(); ++i)
    std::cout << *i << endl;
  
  cout <<endl<<"----- RAW -----" <<endl;
  this->dataRaw->print();
  cout <<endl<<"----- ODOMETRY -----" <<endl;
  this->data->print();
}



Encoder::~Encoder(){
 delete this->dataRight;
 delete this->dataLeft;
 delete this->dataSynchro;
 delete this->dataRaw; 
}



/*! load raw data to treat*/
int Encoder::loadRaw(string path)
{
  string filename = path+"/"+ this->dataRawFilename;
  if(this->dataRaw->load(filename)<0) 
    return 0;
  
  this->splitData();
  this->synchronizeData();
  return 1;
}

/*! load odom data*/
int Encoder::load(string path)
{
  string filename = path+"/"+ this->dataOdometryFilename;
  if(this->data->load(filename)<0) 
    return 0;
  return 1;
}



int Encoder::save(string path){
  string filename = path+"/"+ this->dataOdometryFilename;
  return this->data->save(filename,17);
}
int Encoder::saveLeft(string path){
  string filename = path+"/"+ this->dataLeftFilename;
  return this->dataLeft->save(filename,17);
}

int Encoder::saveRaw(string path){
  string filename = path+"/"+ this->dataRawFilename;
  return this->dataRaw->save(filename,17);
}

int Encoder::saveRight(string path){
  string filename = path+"/"+ this->dataRightFilename;
  return this->dataRight->save(filename,17);
}


int Encoder::saveSynchro(string path){
  string filename = path+"/"+ this->dataSynchroFilename;
  return this->dataSynchro->save(filename,17);
}


/*!
 * 
 * Split right and left data from data read in a file
 * 
 * Warning : suppose that data has been filled
 * #1:absTime #2:relPulse #3:relDist #4:absPulse #5:absDist
 *
 */
int Encoder:: splitData(){ 
  
 //maximum data in right and left is the total number of data 
 int maxRow(this->dataRaw->getRows()); 
 
 if (maxRow==0)
 {
    std::cerr << "ERR:SplitData >> no data to split" << std::endl;
    return -1;  
 } 
  
  
 this->dataLeft->resize(maxRow,5);
 this->dataRight->resize(maxRow,5);
 
 double wheelSize(0.1);
 int indexLeft(0), indexRight(0); 

 // run on all the file lines
 for(int i=0;i<maxRow;i++)
  {     
    
     
    // encoder pulse value since the beginning of the experiment
    double absPulse ( this->dataRaw->data(i,1) );
    double absAngle ( absPulse/4*M_PI/180 );
    double absDist  ( absAngle/4*wheelSize );    

    // encoder relative value since the last sample
    double relPulse ( this->dataRaw->data(i,2) );
    double relAngle ( relPulse/4*M_PI/180 );
    double relDist  ( relAngle/4*0.1 );

    
    // time
   // double absTime  ( this->dataRaw->data(i,6) ) ;
   double absTime  ( this->dataRaw->data(i,4) ) ;
    
    // encoder number
    int num(this->dataRaw->data(i,0));   

    // if the encoder is 0, then store data in left matrice else right
     if (num==0)
     { 
       this->dataLeft->data(indexLeft,0) = absTime;
       this->dataLeft->data(indexLeft,1) = relPulse;
       this->dataLeft->data(indexLeft,2) = relDist;
       this->dataLeft->data(indexLeft,3) = absPulse;
       this->dataLeft->data(indexLeft,4) = absDist;
       indexLeft++;
    }
    else if(num==1)
    {
       this->dataRight->data(indexRight,0) = absTime;
       this->dataRight->data(indexRight,1) = -relPulse;
       this->dataRight->data(indexRight,2) = -relDist;
       this->dataRight->data(indexRight,3) = -absPulse;
       this->dataRight->data(indexRight,4) = -absDist;
       indexRight++;
    }
    else 
    {
      std::cerr << "Error encoder number" << std::endl;
      return -1;   
    }
   }
     
   this->dataLeft->setRows(indexLeft);
   this->dataRight->setRows(indexRight);
  
   
}


/*
* Build a folder with left and right data synchronised
* Input  dataSynchro->data : #0:absTime #1:relPulse  #2:relDist  #3:absPulse  #4:absDist
* Output dataSynchro->data : ZERO-BLOC
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
  int col(9);
  int row(this->dataRaw->getRows()); 
  this->dataSynchro->resize(row, col);
  int iterLeft(0);
  int iterRight(0);
  int iter(0);

  double timeLeft  = this->dataLeft->data(iterLeft,0);
  double timeRight = this->dataRight->data(iterRight,0);

  bool finished(false);
  while(!finished)
  {  
     //cout << iter <<" :raw: "<< dataRaw->getRows() << " | "
     //     << iter <<" :sync: "<< dataSynchro->getRows() << " | "
     //     << iterLeft <<" :left: "<< dataLeft->getRows()<< " | "
     //     << iterRight <<" :righ: "<< dataRight->getRows()<<  endl;
        // if the two iterators are smaller than dataSynchro->data size
	if( iterLeft<this->dataLeft->getRows() && iterRight<this->dataRight->getRows())
	{ 
    //std::cout << "both smaller :time left: " <<setprecision(15)<<timeLeft<< " :time Right: " << setprecision(15)<<timeRight<< std::endl;
		//if the two times are the same
		if(timeLeft==timeRight)
		{   

			this->dataSynchro->data(iter,0) = timeLeft;
			this->dataSynchro->data(iter,1) = this->dataLeft->data(iterLeft,1);
			this->dataSynchro->data(iter,2) = this->dataLeft->data(iterLeft,2);
			this->dataSynchro->data(iter,3) = this->dataLeft->data(iterLeft,3);
			this->dataSynchro->data(iter,4) = this->dataLeft->data(iterLeft,4);

			this->dataSynchro->data(iter,5) = this->dataRight->data(iterRight,1);
			this->dataSynchro->data(iter,6) = this->dataRight->data(iterRight,2);
			this->dataSynchro->data(iter,7) = this->dataRight->data(iterRight,3);
			this->dataSynchro->data(iter,8) = this->dataRight->data(iterRight,4);

			iterLeft++;
			iterRight++;
      timeLeft       = this->dataLeft->data(iterLeft,0);
      timeRight      = this->dataRight->data(iterRight,0);
		}
		else if(timeLeft<timeRight)
		{
      //std :: cout << "tl<tr"<<std::endl;
      this->dataSynchro->data(iter,0) = timeLeft;
			this->dataSynchro->data(iter,1) = this->dataLeft->data(iterLeft,1);
			this->dataSynchro->data(iter,2) = this->dataLeft->data(iterLeft,2);
			this->dataSynchro->data(iter,3) = this->dataLeft->data(iterLeft,3);
			this->dataSynchro->data(iter,4) = this->dataLeft->data(iterLeft,4);
			this->dataSynchro->data(iter,5) = 0;
			this->dataSynchro->data(iter,6) = 0;
      
      if(iterRight>0){
        
				this->dataSynchro->data(iter,7) = this->dataRight->data(iterRight-1,3);
        this->dataSynchro->data(iter,8) = this->dataRight->data(iterRight-1,4);
			}
			iterLeft++;
      if(iterLeft<this->dataLeft->getRows()) //update
          timeLeft   = this->dataLeft->data(iterLeft,0);
		}
		else if (timeRight<timeLeft)
		{

      this->dataSynchro->data(iter,0) = timeRight;

			this->dataSynchro->data(iter,1) = 0;
			this->dataSynchro->data(iter,2) = 0;

			if(iterLeft>0){
				this->dataSynchro->data(iter,3) = this->dataLeft->data(iterLeft-1,3);
				this->dataSynchro->data(iter,4) = this->dataLeft->data(iterLeft-1,4);
			}

			this->dataSynchro->data(iter,5) = this->dataRight->data(iterRight,1);
			this->dataSynchro->data(iter,6) = this->dataRight->data(iterRight,2);
			this->dataSynchro->data(iter,7) = this->dataRight->data(iterRight,3);
			this->dataSynchro->data(iter,8) = this->dataRight->data(iterRight,4);

			iterRight++;
      if(iterRight<this->dataRight->getRows())
            timeRight      = this->dataRight->data(iterRight,0);
		}		
	}           
	else if( iterLeft<this->dataLeft->getRows() && iterRight>=this->dataRight->getRows())
	{
   //   std::cout << "l smaller and r >=" << std:: endl;
      this->dataSynchro->data(iter,0) = timeLeft;
			this->dataSynchro->data(iter,1) = this->dataLeft->data(iterLeft,1);
			this->dataSynchro->data(iter,2) = this->dataLeft->data(iterLeft,2);
			this->dataSynchro->data(iter,3) = this->dataLeft->data(iterLeft,3);
			this->dataSynchro->data(iter,4) = this->dataLeft->data(iterLeft,4);

			this->dataSynchro->data(iter,5) = 0;
			this->dataSynchro->data(iter,6) = 0;
      if(iterRight>0){
				this->dataSynchro->data(iter,7) = this->dataRight->data(iterRight-1,3);
		        	this->dataSynchro->data(iter,8) = this->dataRight->data(iterRight-1,4);
			}
		iterLeft++;
  		if(iterLeft<this->dataLeft->getRows())
      timeLeft       = this->dataLeft->data(iterLeft,0);
	}
	else  if( iterLeft>=this->dataLeft->getRows() && iterRight<this->dataRight->getRows())
	{
     // std::cout << "l <= and r>" << std:: endl;

      this->dataSynchro->data(iter,0) = timeRight;
			this->dataSynchro->data(iter,1) = 0;
			this->dataSynchro->data(iter,2) = 0;
			if(iterLeft>0){
				this->dataSynchro->data(iter,3) = this->dataLeft->data(iterLeft-1,3);
				this->dataSynchro->data(iter,4) = this->dataLeft->data(iterLeft-1,4);
			}
			this->dataSynchro->data(iter,5) = this->dataRight->data(iterRight,1);
			this->dataSynchro->data(iter,6) = this->dataRight->data(iterRight,2);
			this->dataSynchro->data(iter,7) = this->dataRight->data(iterRight,3);
			this->dataSynchro->data(iter,8) = this->dataRight->data(iterRight,4);
		  iterRight++;
  		
      if(iterRight<this->dataRight->getRows())
      timeRight      = this->dataRight->data(iterRight,0);
	}
	else 
	{
		cout << " ERREUR" << endl;
		cout << iter << " " <<  endl 
                             << " iterL : " << iterLeft << "et le max" << this->dataLeft->getRows() 
		             << " iterR : " << iterRight<< "et le max" << this->dataRight->getRows() << endl ;
	}	
	iter ++;
	if(iter>=this->dataSynchro->getRows()) finished = true; 
  }
  
  return 0;
  
}





//function [x,y,theta] = computeWalkerPosition(time, delta0, delta1, L)
/*
 * dataSynchro->data :
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
   std::cout << "Fonction odometry" << std::endl;
   
   // test parameters
   if(L<=0) return -1;
  
  
   // test nbSample in sync data
   int nbSamples(this->dataSynchro->getRows());
   if(nbSamples <=0) return -2;
  
   std::cout << "Test ok suite" << std::endl;
       
   // resize the output data    
   this->data->resize(nbSamples,4);
       
   std::cout << "redim ok " << std::endl;
      
   // init the variables
   double dOmega(0), dS(0), deltaRight(0), deltaLeft(0);
   double xprec(0), yprec(0), thetaprec(0),x(0),y(0),theta(0) ;
  
   for(int i=0; i< nbSamples;i++)
   {
           deltaRight = this->dataSynchro->data(i,6) ;
           deltaLeft  = this->dataSynchro->data(i,2) ;
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
	   this->data->data(i,0) =  this->dataSynchro->data(i,0);
     this->data->data(i,1) = x;
	   this->data->data(i,2) = y;	   	
	   this->data->data(i,3) = theta;
	   
     // update prec
     xprec = x ;
     yprec = y ;
	   thetaprec = theta ; 		           		
        }
  
     return  0;

}




int CCONV Encoder::attachHandler(CPhidgetHandle ENC, void *userptr)
{
        int serialNo;
        CPhidget_DeviceID deviceID;
        int i, inputcount;

        CPhidget_getSerialNumber(ENC, &serialNo);

        //Retrieve the device ID and number of encoders so that we can set the enables if needed
        CPhidget_getDeviceID(ENC, &deviceID);
        CPhidgetEncoder_getEncoderCount((CPhidgetEncoderHandle)ENC, &inputcount);
        printf("Encoder %10d attached! \n", serialNo);

        //the 1047 requires enabling of the encoder inputs, so enable them if this is a 1047    
        if (deviceID == PHIDID_ENCODER_HS_4ENCODER_4INPUT)
        {
                printf("Encoder requires Enable. Enabling inputs....\n");
                for (i = 0 ; i < inputcount ; i++)
                        CPhidgetEncoder_setEnabled((CPhidgetEncoderHandle)ENC, i, 1);
        }
        return 0;
}


int CCONV Encoder::detachHandler(CPhidgetHandle ENC, void *userptr)
{
	int serialNo;
	CPhidget_getSerialNumber(ENC, &serialNo);
	printf("Encoder %10d detached! \n", serialNo);

	return 0;
}

int CCONV Encoder::errorHandler(CPhidgetHandle ENC, void *userptr, int ErrorCode, const char *Description)
{
	printf("Error handled. %d - %s \n", ErrorCode, Description);
	return 0;
}

int CCONV Encoder::inputChangeHandler(CPhidgetEncoderHandle ENC, void *usrptr, int Index, int State)
{
  printf("Input #%i - State: %i \n", Index, State);
	return 0;
}

int CCONV Encoder::positionChangeHandler(CPhidgetEncoderHandle ENC, void *usrptr, int Index, int Time, int RelativePosition)
{ 
  int Position;
	CPhidgetEncoder_getPosition(ENC, Index, &Position);
  printf("Encoder #%i - Position: %5d -- Relative Change %2d -- Elapsed Time: %5d \n", Index, Position, RelativePosition, Time);
	printf("%i\t %5d\t%2d\t%5d\n", Index, Position, RelativePosition, Time);
	
  ((std::vector<double>*)usrptr)->push_back(Index);
  ((std::vector<double>*)usrptr)->push_back(Position);
  ((std::vector<double>*)usrptr)->push_back(RelativePosition);
  ((std::vector<double>*)usrptr)->push_back(Time);
  
  std::cout << "Buffer Size"<<((std::vector<double>*)usrptr)->size() << std::endl;
  
  
  return 0;
}

//Display the properties of the attached phidget to the screen.  We will be displaying the name, serial number and version of the attached device.
//Will also display the number of inputs and encoders on this device
int Encoder::displayProperties(CPhidgetEncoderHandle phid)
{
	int serialNo, version, num_inputs, num_encoders;
	const char* ptr;

	CPhidget_getDeviceType((CPhidgetHandle)phid, &ptr);
	CPhidget_getSerialNumber((CPhidgetHandle)phid, &serialNo);
	CPhidget_getDeviceVersion((CPhidgetHandle)phid, &version);

	CPhidgetEncoder_getInputCount(phid, &num_inputs);
	CPhidgetEncoder_getEncoderCount(phid, &num_encoders);

	printf("%s\n", ptr);
	printf("Serial Number: %10d\nVersion: %8d\n", serialNo, version);
	printf("Num Encoders: %d\nNum Inputs: %d\n", num_encoders, num_inputs);

	return 0;
}


}

