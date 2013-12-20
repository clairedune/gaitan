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

#ifndef ENCODER_H
#define ENCODER_H
#include <string>

#include <libgaitan/sensor.h>
#include <libgaitan/table.h>
using namespace std;

namespace gaitan
{
class Encoder: public Sensor
{

  private:
    Table *_dataRaw; // data file raw de chez raw
    Table *_dataLeft;
    Table *_dataRight;
    Table *_dataSynchro; // reorganise and synchronise data

    
  public:	
    Encoder();
    Encoder(string filename); 
    ~Encoder();

    int readFile(string filname);
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
