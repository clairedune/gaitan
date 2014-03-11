
#include <libgaitan/shape.h>
#include<Eigen/Dense>

#include<iostream>

using namespace Eigen;

namespace gaitan
{
     Shape::Shape(){}
    
     Shape::Shape(const VectorXf & param) : parameters(param){}
     
     Shape::~Shape(){}
     
     
     Eigen::VectorXf Shape::getParameters(){return this->parameters;}
     
     void Shape::print(){
		        std::cout <<"The shape parameters are:\n" << this->parameters.transpose() << std::endl;
		 }
     
     /*
      * FindParameters : fit the 3D parametric shape to the pts
      * set the parameters of the shape to the fitted one
      */
     int Shape::findParameters(const Eigen::MatrixXf & pts ){
		 
		 return 1;
		 }
     
     /*
      * Compute the distance between a 3D point and the shape
      * return a float.
      */
     float Shape::computeDistance(const double &X, const double &Y, const double & Z){
		 
		 return 1;
		 }
      
    /*
     * Compute the 
     */
     Eigen::VectorXf Shape::computeDistance(const Eigen::MatrixXf & pts){

		 // defaut values
		 Eigen::VectorXf dist(1);
		 dist(0) = -1;
		 
		 
		 return dist;
	 }
}
