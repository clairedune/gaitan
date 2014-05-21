
#include <libgaitan/geometry.h>
#include <Eigen/Dense>
#include <iostream>


#include <visp/vpConfig.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpRotationMatrix.h>


namespace gaitan
{
    
 int Geometry::homogeneousMatrix (double& x, double& y, double& theta, Eigen::MatrixXf & wMg)
{
   //rotation
   vpRxyzVector rotVector(0,0,theta);
   vpRotationMatrix rotMatrix(rotVector);
   
   //translation
   vpTranslationVector transVector(x,y,0); 
  
   //homogeneousMatrix
   vpHomogeneousMatrix M(transVector, rotMatrix);
   
   //eigen Matrix
   
   
   for (int i=0 ; i<4 ;  i++)
     for (int j=0 ; j<4 ; j++)
     {
        wMg(i,j)=M[i][j];    
     }
   
   return 1;
  
}

   
}
