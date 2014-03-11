#ifndef GAITAN_BOX_H
#define GAITAN_BOX_H

#include <iostream>
#include <stdlib.h>
#include <vector>

#include <libgaitan/shape.h>
#include <Eigen/Dense>


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
                            
     inline void setParameters(const VectorXf& param){this->parameters=param;}     
          
     inline double getMinX(){return this->parameters(0);}
     inline double getMaxX(){return this->parameters(1);}
     inline double getMinY(){return this->parameters(2);}
     inline double getMaxY(){return this->parameters(3);}
     inline double getMinZ(){return this->parameters(4);}
     inline double getMaxZ(){return this->parameters(5);}
     
     void print(); 
     
     int inlierSelection( Eigen::MatrixXf & ptsIn, 
                          Eigen::MatrixXf & ptsOut, 
                          double & distThreshold);
     
     static int inlierSelection( std::vector<Box>& boxes,
                                 Eigen::MatrixXf & ptsIn, 
                                 Eigen::MatrixXf & ptsOut, 
                                 double & distThreshold);
     
     bool isInBox(float x, float y, float z, float S);
     
     
     /*!compute the center of the box*/
     inline int center(float &x, float &y, float &z)
     {
       if (parameters.size()==6){
       x=(this->parameters(0)+this->parameters(1))/2;
       y=(this->parameters(2)+this->parameters(3))/2;
       z=(this->parameters(4)+this->parameters(5))/2; 
       return 1;}
       else
      return -1;
      }
     
     /*!Compute the volume of the box*/
     inline float volume()
     {
       if (parameters.size()==6){
       return (this->parameters(1)-this->parameters(0))*
              (this->parameters(3)-this->parameters(2))*
              (this->parameters(5)-this->parameters(4)) ;
       }
       else
      return 0;
      }
     
     /*!Distance between two boxes*/ 
     static float distance(Box &  box1, Box & box2);
      
      
      
     int findParameters(const MatrixXf & pts );
     VectorXf computeDistance(const MatrixXf & pts);
     float computeDistance(const double &X, const double &Y, const double & Z);
      
  };
}
#endif // PLANE_H
   
