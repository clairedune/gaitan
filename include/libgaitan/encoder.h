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

#ifndef GAITAN_ENCODER_H
#define GAITAN_ENCODER_H
#include <string>

#include <iostream>
#include <vector> 
#include <libgaitan/sensor.h>
#include <libgaitan/table.h>
#include <libgaitan/rgbdsensor.h>
#include <phidget21.h>

using namespace std;

namespace gaitan
{
  class Encoder: public Sensor
  {

  private:
    Table *dataRaw;  // data file as acquired by the phidget provided code
    Table *dataLeft; // data corresponding to the left encoder
    Table *dataRight; // data corresponding to the right encoder
    Table *dataSynchro; // reorganise and synchronise left and right data
    //Table *data ; // In Sensor class contains the odometry data

    
    // name of the filename for the encoder matrices
    std::string dataRawFilename ; 
    std::string dataLeftFilename ; 
    std::string dataRightFilename ; 
    std::string dataSynchroFilename ; 
    std::string dataOdometryFilename ; 
     
    
   public:	
   
    vector<double> buffer; // buffer to acquire data with phidget handler
   
    Encoder();
    Encoder(string filename); 
    ~Encoder();

    /*! Load the odom data contained in a file*/
    int load(string filename);
    
    /*! Load the raw data contained in a file*/
    int loadRaw(string filename);
    
    //int save(string filename, int precision);
    
    /*! When the data are acquired, they are stored in a buffer that need to be flushed
     * in the appropriate tables : first Raw, then split in right and left and synchronised */
    int flush(double initTime=0);
    
    /*! Acquire data and store them in a file*/
    int acquire(const std::string &path , bool flagDisp=true);
    
    /*! Acquire data and store them in a buffer*/
    int acquire();
    void print(int beg, int end);
    void print();
    void initData();
    int odometry(double); 
    
    int save(string filename);
    int saveLeft(string filename);
    int saveRaw(string filename);
    int saveRight(string filename);
    int saveSynchro(string filename);
    
    //phidget
    static int attachHandler(CPhidgetHandle ENC, void *userptr);
    static int detachHandler(CPhidgetHandle ENC, void *userptr);
    static int errorHandler(CPhidgetHandle ENC, void *userptr, int ErrorCode, const char *Description);
    static int inputChangeHandler(CPhidgetEncoderHandle ENC, void *usrptr, int Index, int State);
    static int positionChangeHandler(CPhidgetEncoderHandle ENC, void *usrptr, int Index, int Time, int RelativePosition);
    int displayProperties(CPhidgetEncoderHandle phid);
   
 
  private  :
    int splitData();
    int synchronizeData();

 
};
}
#endif // ENCODER_H
