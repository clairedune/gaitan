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

/* 
 * pts is a Nx3 matrix each line is the coord of a 3D point (x,y,z)
 * ax+by+cz+d=0
 * d is fixed to -1 
 * apply svd to find the plane parameters
 * 
 */
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
 * d = (ax+by+cz+d)/sqrt(a^2+b^2+c^2)
 */
float absDistance(const Eigen::Vector4f &pt,const Eigen::Vector4f &param){   
   //defaut distance
   float distance(-1);
   
   // compute distance
   float prod = fabs(pt.transpose()*param);
   float div = sqrt(param(0)*param(0)+param(1)*param(1)+param(3)*param(3));
   if (div !=0) distance = prod/div;
  
   // return the computed distance
   return distance;
}

Eigen::VectorXf absDistanceVect(const Eigen::MatrixXf &pts, const Eigen::Vector4f &param){

     // define a vector to store the distances
     // same size as the number of points stored in the matrix 
     Eigen::VectorXf dist(pts.rows());
     Eigen::VectorXf pt;
     
     // compute the distance for all thoses points
     for (int i=0; i< pts.rows() ; i++){
         pt =  pts.block(i,0,1,3).transpose();
         pt.conservativeResize(4);
         pt(3)=1;
         dist(i) = absDistance(pt,param);
     }
     
     return dist;
}


int main() {
	
      // real plane coeff
      double  a (0.0),
              b (0.0),
              c (1),
              d (-1);
       
      Eigen::Vector4f paramReal(a,b,c,d); 
       
      std::cout << "The real plane coeff are:\n" << a << " "<< b << " " << c << " "<< d << std::endl; 

      // create 100 points matrix that belong to the plane
      int nbPts(100);
      Eigen::MatrixXf pts(nbPts,3);
      for (int i=0; i<nbPts;i++){
          pts(i,0) = rand()%20 - 10;
          pts(i,1) = rand()%20 - 10;
          pts(i,2) = -(a*pts(i,0)+b*pts(i,1)+d)/c+0.01*(rand()%20-10);
      }
   
    std::cout << "Les points "<< pts.block(0,0,10,3) <<std::endl;   
    Eigen::VectorXf param ;
    findParameters(pts,param);
    std::cout << "The least-squares solution is:\n";
    std::cout << param.transpose() << std::endl;
    
    //on prend la distance avec le premier point
    Eigen::VectorXf dist = absDistanceVect(pts,paramReal);
    std::cout << "distance : "<<dist.block(0,0,10,1) << std::endl; 
    
    return 1;      
}
