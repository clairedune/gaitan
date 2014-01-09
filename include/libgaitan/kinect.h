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
      
  };
}
#endif // kinect_H
