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

#ifndef TABLE_H
#define TABLE_H
#include <string>

#include <Eigen/Dense>
using namespace std;

namespace gaitan
{
class Table
{
  public:
    double** _data; // data file  
    int _nbRow;    // data number
    int _nbCol;    // data type  
    
   // Eigen::MatrixXf _data;
    
  public:	
    Table();
    Table(int col, int raw);
    ~Table();
    int createFromFile(string filname);
    void print(int beg, int end);
    void print();
    void resize(int col, int raw);
    int writeInFile(string filename, int precision);
    void init();
};
}

#endif // ENCODER_H
