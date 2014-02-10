/*!
 * Test the plane class 
 */

#include <iostream>
#include <Eigen/Dense>
#include <libgaitan/plane.h>

using namespace gaitan;



int main() {

      // real plane coeff
      double  a (2.0),
              b (1.4),
              c (2),
              d (-1);
              
     Plane plane(a,b,c,d);
     plane.print();
     
     // create 100 points matrix that belong to the plane
     int nbPts(200);
     Eigen::MatrixXf pts(nbPts,3);
     for (int i=0; i<nbPts;i++){
          pts(i,0) = rand()%20 - 10;
          pts(i,1) = rand()%20 - 10;
          if(i < nbPts*20/100) // 20% noisy
             pts(i,2) = -(a*pts(i,0)+b*pts(i,1)+d)/c+0.01*(rand()%20-10);
          else // 80% perfect
             pts(i,2) = -(a*pts(i,0)+b*pts(i,1)+d)/c;
      }
   
    std::cout << "Les points "<< pts.block(0,0,10,3) <<std::endl;   
     
    // create a plane to estimate the parameters 
    Plane estPlane; 
    estPlane.findParameters(pts); 
    estPlane.print();   
    
    // recursive plane fitting and outliers selection
    Eigen::MatrixXf ptsIn(pts), ptsOut(3,0);
    double confidence(0.03);
  // estPlane.findParameters(ptsIn, ptsOut, confidence);
    //estPlane.print(); 
    
     
    return 1;      
}
