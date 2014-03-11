#ifndef GAITAN_KINECT_H
#define GAITAN_KINECT_H

#include <libgaitan/rgbdsensor.h>

#include <Eigen/Dense>

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>

#ifdef __GNUC__
#define DEPRECATED __attribute__((deprecated))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#else
#pragma message("WARNING: You need to implement DEPRECATED for this compiler")
#define DEPRECATED
#endif

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
    
    DEPRECATED Eigen::MatrixXf pointCloud(const std::string & path,const int &index); // deprecated
  
    int pointCloud(const std::string & path,const int &index, Eigen::MatrixXf & pts3D);

    
  };
}
#endif // RGBDSensor_H
