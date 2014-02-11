#ifndef GAITAN_PLANE_H
#define GAITAN_PLANE_H

#include<libgaitan/shape.h>
#include<Eigen/Dense>

#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>



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
     Plane(Plane & plane);
     ~Plane();
          
    Plane & operator = (Plane &);      
          
     double  getA();
     double  getB();
     double  getC();
     double  getD();
          
     virtual int findParameters(const MatrixXf& pts );
     virtual VectorXf computeDistance(const MatrixXf& pts);
     virtual VectorXf computeError(const MatrixXf& pts);
     
     void print(); 
     
     
     int findParameters(Eigen::MatrixXf & ptsIn, 
                        Eigen::MatrixXf & ptsOut, 
                        double & distThreshold);
                        
     int findParameters(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, double & distanceThreshold);
                   
    
     int inlierSelection( Eigen::MatrixXf & ptsIn, 
                          Eigen::MatrixXf & ptsOut, 
                          double & distThreshold);
    
     // fixme : can t see the points created using pcl. May be to small part of space ?
     void createPointCloud( Eigen::MatrixXf & pts);
     
     // compute the transformation between the plane and the camera
     Eigen::Matrix3f computeOrientation();
     Eigen::Matrix4f computeTransformation();

      
     void changeFrame(const Eigen::Matrix4f & cMo) ;
     
     
    protected: 
     virtual float computeDistance(const double &X, const double &Y, const double & Z);
     virtual float computeError(const double &X, const double &Y, const double & Z);

      
  };
}
#endif // PLANE_H
   
