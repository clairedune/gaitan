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


#include <libgaitan/inertia.h>
#include <libgaitan/table.h>

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <math.h>


//#define M_PI 3.1415


using namespace std;

namespace gaitan
{
Inertia::Inertia(){
   this->_data = new Table(10,10); 
   this->initData();
}

Inertia::Inertia(string filename)
{
  this->_data = new Table(10,10);
  this->_data->createFromFile(filename);
  
}

Inertia::~Inertia(){
// delete this->_data; 
}

void Inertia::print(int a, int b)
{
   cout << endl<<"-------- INERTIA -------" << endl;
   this->_data->print(a,b);
}

/*int Inertia::readFile(string filename){
  this->_data->createFromFile(filename);
  return 0;
}*/

}

