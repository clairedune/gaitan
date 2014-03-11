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
    //double** _data; // data file  
    Eigen::MatrixXd data;
    
    
  private:   
    int nbRow;    // data number
    int nbCol;    // data type  
    
    
  public:	
    Table();
    Table(int col, int raw);
    ~Table();
    
    inline int rows(){return data.rows();}
    inline void rows(const int & r){return data.conservativeResize(nbCol,r); nbRow=r;}
    inline int cols(){return data.cols();}
    inline void cols(const int & c){return data.conservativeResize(c,nbRow); nbCol=c;}

    
    
    int  load(string filname);
    void print(int beg, int end);
    void print();
    void resize(int col, int raw);
    int  save(string filename, int precision);
    void init();
};
}

#endif // ENCODER_H
