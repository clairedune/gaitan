/*!
 * Read the kinect data saved in a directory 
 * given as an input to the test
 * 
 * 
 * 
 */


#include <visp/vpConfig.h>

#include <sys/time.h> // to get time
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <math.h>
#include <cstdlib>


#include <Eigen/Dense>

/* pts is a Nx3 matrix each line is the coord of a 3D point (x,y,z)
* ax+by+cz+d=0
* d is fixed to -1 
* apply svd to find the plane parameters
* */
int findParameters(const Eigen::MatrixXf & pts, Eigen::VectorXf & param){
  double d=-1;
  int nbPts = pts.rows();
  Eigen::VectorXf res(nbPts);
  for (int i=0; i<nbPts;i++){
          res(i) = -d;
  }
  param = pts.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(res);
  param.conservativeResize(4);
  param(3)=d;
  return 1;
}

/*
 * Compute absolute distance 
 * pt = (x,y,z) 
 */
float absDistance(const Eigen::Vector3f &pt,const Eigen::VectorXf &param)
{
   float distance (-1);
   float prod = pt*param;
    
   distance = prod;
   return distance;
}
int main() {
	
      // real plane coeff
      double  a (2.0),
              b (3.0),
              c (2),
              d (-1);
       
      std::cout << "The real plane coeff are:\n" << a << " "<< b << " " << c << " "<< d << std::endl; 

      // create 100 points matrix that belong to the plane
      int nbPts(1000);
      Eigen::MatrixXf pts(nbPts,3);
      for (int i=0; i<nbPts;i++){
          pts(i,0) = rand()%20 - 10;
          pts(i,1) = rand()%20 - 10;
          pts(i,2) = -(a*pts(i,0)+b*pts(i,1)+d)/c;//+0.01*(rand()%20-10);
      }
   
    //std::cout << "Les points "<< pts << "et res" << res<<std::endl;   
    Eigen::VectorXf param ;
    findParameters(pts,param);
    std::cout << "The least-squares solution is:\n";
    std::cout << param.transpose() << std::endl;
    
    float dist = absDistance(pt(i),const Eigen::VectorXf &param)
    
    return 1;      
}
