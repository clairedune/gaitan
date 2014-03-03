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

#include <libgaitan/sensor.h>
#include <libgaitan/table.h>
using namespace std;

namespace gaitan
{
  class Encoder: public Sensor
  {

  private:
    Table *_dataRaw;  // data file as acquired by the phidget provided code
    Table *_dataLeft; // data corresponding to the left encoder
    Table *_dataRight; // data corresponding to the right encoder
    Table *_dataSynchro; // reorganise and synchronise left and right data

    
  public:	
    Encoder();
    Encoder(string filename); 
    ~Encoder();

    int readFile(string filname);
    int acquire(const std::string &path , bool flagDisp=true);
    void print(int beg, int end);
    void print();
    void initData();
    int odometry(double); 
    int writeInFileLeft(string filename);
    int writeInFileRight(string filename);
    int writeInFileSynchro(string filename);
 
  private  :
    void splitData();
    int  synchronizeData();
    
 
};
}
#endif // ENCODER_H
