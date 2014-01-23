#ifndef GAITAN_SHAPE_H
#define GAITAN_SHAPE_H


#include<Eigen/Dense>

using namespace Eigen;

namespace gaitan
{
  class Shape
  {  
	private:
	vectorXf parameters;
	
    public:	
     Shape();
     virtual ~Shape();
     
     vectorXf getParameters();
     void print();
     
     /*
      * FindParameters : fit the 3D parametric shape to the pts
      * set the parameters of the shape to the fitted one
      */
     virtual int findParameters(const matrixXf & pts );
     virtual float computeDistance(const vector3f & pt);
     virtual vectorXf computeDistance(const matrixXf & pts)
      
  };
}
#endif // shape_H
