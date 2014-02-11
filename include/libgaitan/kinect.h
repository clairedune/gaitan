#ifndef GAITAN_KINECT_H
#define GAITAN_KINECT_H

#include <libgaitan/sensor.h>
#include <libgaitan/table.h>



namespace gaitan
{
  class Kinect:public Sensor
  {   
    public:	
     Kinect();
     Kinect(string path);
     ~Kinect();
    
    int readFile(string filename);
    int writeInFile(string filename, int precision=15);
    virtual void print(int beg, int end);
    virtual void print();
    void initData();
    virtual int acquire();
    
  };
}
#endif // kinect_H
