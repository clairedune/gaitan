
#include <gaitan/shape.h>
#include<Eigen/Dense>

using namespace Eigen;

namespace gaitan
{
     Shape::Shape(){}
     
     vectorXf getParameters(){returns this->parameters};
     
     void Shape::Sprint(){
		 
		 std::cout <<"The shape parameters are:\n" << parameters << std::endl;
		 
		 }
     
     /*
      * FindParameters : fit the 3D parametric shape to the pts
      * set the parameters of the shape to the fitted one
      */
     int Shape::SfindParameters(const matrixXf & pts ){
		 
		 return 1;
		 }
     
     /*
      * Compute the distance between a 3D point and the shape
      * return a float.
      */
     float Shape::computeDistance(const vector3f & pt){
		 
		 return 1;
		 }
      
    /*
     * Compute the 
     */
     vectorXf Shape::computeDistance(const matrixXf & pts){
		 
		 return 1;
		 }  
  };
}
#endif // shape_H
