#ifndef GAITAN_KINECT_H
#define GAITAN_KINECT_H

#include <libgaitan/rgbdsensor.h>

#include <Eigen/Dense>

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>


namespace gaitan
{
  class Kinect:public RGBDSensor
  {   
    public:	
      Kinect();
     ~Kinect();
      Kinect(double fx, double fy, double cx, double cy);
    
    
    int display();
    int acquire(const std::string &path , bool flagDisp=true);
    Eigen::MatrixXf pointCloud(const std::string & path,const int &index);

    
  };
}
#endif // RGBDSensor_H
