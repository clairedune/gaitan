#ifndef GAITAN_BOX_H
#define GAITAN_BOX_H

#include<libgaitan/shape.h>
#include<Eigen/Dense>

// TODO : surcharger les operation << et =
//

using namespace Eigen;

namespace gaitan
{
  class Box:public Shape
  {  
	  public:	
     
     Box();
     
     Box(const double& minx, const double& maxx, 
         const double& miny, const double& maxy,
         const double& maxz, const double& minz);
         
     Box(const VectorXf& param);
     
     ~Box();
             
     //Box& operator= (Box box);      
      
     void setParameters(const double& minx, const double& maxx, 
                        const double& miny, const double& maxy,
                        const double& maxz, const double& minz); 
                            
     void setParameters(const VectorXf& param);     
          
     double  getMinX();
     double  getMaxX();
     double  getMinY();
     double  getMaxY();
     double  getMinZ();
     double  getMaxZ();
     
     void print(); 
     
     int inlierSelection( Eigen::MatrixXf & ptsIn, 
                          Eigen::MatrixXf & ptsOut, 
                          double & distThreshold);
     
     
     int findParameters(const MatrixXf & pts );
     VectorXf computeDistance(const MatrixXf & pts);
     float computeDistance(const double &X, const double &Y, const double & Z);
      
  };
}
#endif // PLANE_H
   
