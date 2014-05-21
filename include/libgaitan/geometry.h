#ifndef GAITAN_GEOM_H
#define GAITAN_GEOM_H

#include <iostream>

#include <Eigen/Dense>

#include <visp/vpConfig.h>


namespace gaitan
{
  class Geometry
  {  
    
    public: 
    
    static  int homogeneousMatrix (double& x, double& y, double& theta, Eigen::MatrixXf & wMg);

	  
    // change expression frame of a give 3D point x,y, z. Resulting point is xout, yout, zout 
    static int changeFrame(const double& x, 
                           const double& y, 
                           const double& z, 
                           double & xOut, 
                           double & yOut, 
                           double & zOut, 
                           const Eigen::MatrixXf & wMo);
  };
}
#endif 
   
