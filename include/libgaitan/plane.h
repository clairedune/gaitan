#ifndef GAITAN_PLANE_H
#define GAITAN_PLANE_H

#include<libgaitan/shape.h>
#include<Eigen/Dense>


// TODO : surcharger les operation << et =
//

using namespace Eigen;

namespace gaitan
{
  class Plane:public Shape
  {  
	  public:	
     Plane();
     Plane(const double& a, const double& b, const double& c, const double&d);
     Plane(const Vector4f& param);
     ~Plane();
          
     virtual int findParameters(const MatrixXf& pts );
     
     virtual VectorXf computeDistance(const MatrixXf& pts);
     virtual VectorXf computeError(const MatrixXf& pts);
     
     void print(); 
     
     
     int findParameters(Eigen::MatrixXf & ptsIn, 
                        Eigen::MatrixXf & ptsOut, 
                        const double & distThreshold);
    
     int inlierSelection( Eigen::MatrixXf & ptsIn, 
                          Eigen::MatrixXf & ptsOut, 
                          const double & distThreshold);
    
    protected: 
     virtual float computeDistance(const double &X, const double &Y, const double & Z);
     virtual float computeError(const double &X, const double &Y, const double & Z);

      
  };
}
#endif // PLANE_H
   
