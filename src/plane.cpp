
#include <libgaitan/plane.h>
#include<Eigen/Dense>
#include<iostream>

using namespace Eigen;

namespace gaitan
{
     Plane::Plane(){
		  this->parameters.resize(4);
		  this->parameters(3) =-1; //set d to -1 by default
		 }
		 
     Plane::Plane(const double& a, const double& b, const double& c, const double&d){
       this->parameters.resize(4);
		   this->parameters(0) = a;
       this->parameters(1) = b;
       this->parameters(2) = c;
       this->parameters(3) = d; 
     }
    
     Plane::Plane(const Vector4f& param){
      this->parameters = param;
     }
     
     Plane::~Plane(){}
     
     void Plane::print(){
		 std::cout <<"The shape parameters are:\n" << this->parameters << std::endl;
		 }
		 
     /*!
      * FindParam : 
      * fit the 3D parametric shape to the pts
      * set the parameters of the plane to the fitted one
      */
     int Plane::findParameters(const Eigen::MatrixXf & pts ){
  		
  		// find nb of 3D points in the set
  		int nbPts = pts.rows();
  		
  		// build the res vector for the eq Ax=res
  		Eigen::VectorXf res(nbPts);
  		for (int i=0; i<nbPts;i++){
          res(i) = -this->parameters(3);
  		}	
  
  		this->parameters = pts.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(res);
  		this->parameters.conservativeResize(4);
  		this->parameters(3)=-1;
  		return 1;
  }
     
     /*!
      * Compute the distance between a 3D point and the shape
      * return a float.
      */  
     float Plane::computeDistance(const double &X, const double &Y, const double & Z){
		//defaut distance
   		float distance(-1);
      
      Eigen::Vector4f pt(X,Y,Z,1);
   
   		// compute distance
   		float prod = fabs(pt.transpose()*this->parameters);
   		float div = sqrt(this->parameters(0)*this->parameters(0)
                      +this->parameters(1)*this->parameters(1)
                      +this->parameters(2)*this->parameters(2));
   		if (div !=0) distance = prod/div;
  
   		// return the computed distance
   		return distance;
	}
  
     float Plane::computeError(const double &X, const double &Y, const double & Z){
		//defaut distance
   		float error =this->parameters(0)*X+this->parameters(1)*Y+this->parameters(2)*Z+this->parameters(3) ;
   		return error;
 	   }
  
      
    /*!
     * Compute the distance between a set of 3D points and the shape return distance vector
     */
     Eigen::VectorXf Plane::computeDistance(const Eigen::MatrixXf & pts){

		// define a vector to store the distances
     	// same size as the number of points stored in the matrix 
     	Eigen::VectorXf dist(pts.rows());
     
     	// compute the distance for all thoses points
     	for (int i=0; i< pts.rows() ; i++){
         dist(i) = this->computeDistance(pts(i,0), pts(i,1), pts(i,2));
     	}
     
     	return dist;
	 }
   
   
    Eigen::VectorXf Plane::computeError(const Eigen::MatrixXf & pts){

		// define a vector to store the distances
     	// same size as the number of points stored in the matrix 
     	Eigen::VectorXf error(pts.rows());
     
     	// compute the distance for all thoses points
     	for (int i=0; i< pts.rows() ; i++){
         error(i) = this->computeError(pts(i,0), pts(i,1), pts(i,2));
     	}
     
     	return error;
	 }
   
   /*! 
    * Select the point that are in the plane
    * Divide the set of point into two parts : the inliers and the outliers
    * the ptsIn matrix contains all the points. 
    * the ptsOut matrix contains no point at the beginning
    */
   
int Plane::inlierSelection(Eigen::MatrixXf & ptsIn, 
                     Eigen::MatrixXf & ptsOut, 
                     const double & distThreshold){
   
    // store the current data in a temp matrix
    Eigen::MatrixXf pts(ptsIn);
        
    // compute the distance from data to plane
    Eigen :: VectorXf dist = this->computeDistance(pts);    
        
    // count the number of point in the plane and out of the b
    int nbIn(0), 
        nbOut(0), 
        nbOutPrec(ptsOut.rows()), 
        nbInPrec(ptsIn.rows());
    for (int i=0 ; i< dist.rows(); i++){
      // if the distance btw the 
      // plane and the point is lower than
      // a spectific threhold
      if (dist(i) < distThreshold)  
        nbIn++;
      else 
        nbOut++;
    }
    
    // resize the input matrixes in and out  
    ptsIn.resize(nbIn,3);
    ptsOut.conservativeResize(nbOutPrec+nbOut,3); 

    // select the point in and out
    int indexIn(0), indexOut(0);
    for(int i=0; i<dist.rows(); i++){
      if(dist(i) < distThreshold){
          ptsIn(indexIn,0) = pts(i,0);
          ptsIn(indexIn,1) = pts(i,1);
          ptsIn(indexIn,2) = pts(i,2); 
          indexIn++;
        }
      else{
          ptsOut(nbOutPrec+indexOut-1,0) = pts(i,0);
          ptsOut(nbOutPrec+indexOut-1,1) = pts(i,1);
          ptsOut(nbOutPrec+indexOut-1,2) = pts(i,2); 
          indexOut++;
        }
      }
    }
   
/*! 
 * Recursive method to find parameters and inliers
 * 
 */   
int Plane::findParameters(Eigen::MatrixXf & ptsIn, 
                             Eigen::MatrixXf & ptsOut, 
                             const double & distThreshold){
   
    // store the current data in a temp matrix
    Eigen::MatrixXf pts(ptsIn);
    
    // refine the plane parameters
    this->findParameters(pts); 
    
    // count the number of point  the plane and out of the b
    int nbInPrec(ptsIn.rows());
    
    // select the inliers
    this->inlierSelection(ptsIn, ptsOut, distThreshold);
    
    std::cout << "ptsin " << ptsIn.rows() << std::endl;
    std::cout << "ptsout " << ptsOut.rows() << std::endl;

    // test if the inlier set has evoluated
    if (ptsIn.rows()==nbInPrec){
      return 1; 
    }
    else{
      this->findParameters(ptsIn, ptsOut, distThreshold);
    }
  }
}
