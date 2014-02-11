
#include <libgaitan/plane.h>
#include<Eigen/Dense>
#include<iostream>

#include <pcl/common/common_headers.h>


using namespace Eigen;

namespace gaitan
{
     Plane::Plane(){
		  this->parameters.resize(4);
		  this->parameters(3) =-1; //set d to 1 by default
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
     
     Plane::Plane(Plane & plane){
       Vector4f param = plane.getParameters();
       this->parameters = param;
       }
     
     
     
     Plane::~Plane(){}
     
     
     double Plane::getA(){
       return this->parameters(0);
       }
     double Plane::getB(){
       return this->parameters(1);
       }
     double Plane::getC(){
       return this->parameters(2);
       }
     double Plane::getD(){
       return this->parameters(3);
       }
     
     
     Plane & Plane::operator= (Plane & plane){
          this->parameters = plane.getParameters();
          return *this;    
     }  
     
     
     
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
   		float error = (this->parameters(0)*X+this->parameters(1)*Y+this->parameters(2)*Z+this->parameters(3)) ;
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
                     double & distThreshold){
   
    // store the current data in a temp matrix
    Eigen::MatrixXf pts(ptsIn);
        
    // compute the distance from data to plane
    //Eigen :: VectorXf dist = this->computeDistance(pts); 
    Eigen :: VectorXf dist = this->computeError(pts);    
        
    // count the number of point in the plane and out of the b
    int nbIn(0), 
        nbOut(0), 
        nbOutPrec(ptsOut.rows()), 
        nbInPrec(ptsIn.rows());
    for (int i=0 ; i< dist.rows(); i++){
      // if the distance btw the 
      // plane and the point is lower than
      // a spectific threhold
      if (fabs(dist(i)) < distThreshold)  
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
      if(fabs(dist(i))< distThreshold){
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
                          double & distThreshold){
   
    // refine the plane parameters
    this->findParameters(ptsIn); 
    
    // count the number of point  the plane and out of the b
    int nbInPrec(ptsIn.rows());
    
    // select the inliers
    this->inlierSelection(ptsIn, ptsOut, distThreshold);
    
    // test if the inlier set has evoluated
    if (ptsIn.rows()>=nbInPrec){
      distThreshold /= 4;
      if  (distThreshold < 0.02) return 1; 
      this->findParameters(ptsIn, ptsOut, distThreshold);
      //return 1;
    }
    else{ 
      this->findParameters(ptsIn, ptsOut, distThreshold);
    }
  }
  
/*!
 * 
 * This function finds parameters using pcl algoritm
 *  
 */  
  
int Plane::findParameters(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, double & distanceThreshold)
{
      // create model coefficient
      pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
      // Create the segmentation object
      pcl::SACSegmentation<pcl::PointXYZ> seg;
      // Optional
      seg.setOptimizeCoefficients (true);
      // Mandatory
      seg.setModelType (pcl::SACMODEL_PLANE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setDistanceThreshold (distanceThreshold);

      seg.setInputCloud (cloud->makeShared ());
      seg.segment (*inliers, *coefficients);
      
      std::cerr << "Model coefficients: " << coefficients->values[0] << " " 
                                      << coefficients->values[1] << " "
                                      << coefficients->values[2] << " " 
                                      << coefficients->values[3] << std::endl;  
                                      
                                      
       this->parameters(0) = -coefficients->values[0]/coefficients->values[3];
       this->parameters(1) = -coefficients->values[1]/coefficients->values[3];
       this->parameters(2) = -coefficients->values[2]/coefficients->values[3];
       this->parameters(3) = -coefficients->values[3]/coefficients->values[3];  
       return 1;                                
                                      
}  
  
  
void Plane::createPointCloud( Eigen::MatrixXf & pts)
{
      double a(this->parameters(0));
      double b(this->parameters(1));
      double c(this->parameters(2));
      double d(this->parameters(3));
      int width(100), height(100), sampleStep(0.1), index(0);
      pts.resize(width*height,3);
      for (int i=0; i<width;i++){
        for (int j=0; j<height;j++){
          pts(index,0) = (i-width/2)*sampleStep ;
          pts(index,1) = (i-height/2)*sampleStep ;
          pts(index,2) = -(a*pts(index,0)+b*pts(index,1)+d)/c;
          index++;
      }  
    }
}  
  
  
Eigen::Matrix3f Plane::computeOrientation(){

  Eigen::Matrix3f R =Eigen::MatrixXf::Identity(3,3);
  Eigen::Vector3f e3(this->parameters.block(0,0,3,1));
  e3.normalize();
  //std::cout << e3 << std::endl;

  
  Eigen::Vector3f e2;
  e2 << 0,1,0; 
  //std::cout << e2 << std::endl;
  Eigen::Vector3f e1 (e2.cross(e3)); 
  //std::cout << e1 << std::endl;
  //std:: cout << "norm e1 : " << e1.norm() << std::endl;   
  
  
  R.col(0) = e1;
  R.col(1) = e2;
  R.col(2) = e3;
 // std::cout << R << std::endl;
   
  return R;
}  


/*!
 * 
 * Warning this function compute the transformation to align the
 * frame with the walker frame. That is why the matrix fMw is added.
 * 
 */ 
Eigen::Matrix4f Plane::computeTransformation()
{

      Eigen::Matrix3f kRg = this->computeOrientation();
      Eigen::Matrix4f gMk, wMg (MatrixXf::Identity(4,4));
      gMk.topLeftCorner(3,3) = kRg.inverse() ;
      gMk.topRightCorner(3,1) << 0,0,0 ;
      gMk.bottomLeftCorner(1,3) << 0,0,0;
      gMk.bottomRightCorner(1,1) << 1;
      
      Plane tmp(*this);
      tmp.changeFrame(gMk);
      wMg.topRightCorner(3,1) << 0,0, tmp.getD()/tmp.getC() ;
      
      // to be compatible with the frame of the walker
      Eigen::Matrix4f fMw(MatrixXf::Identity(4,4));
      fMw(1,1)=-1;
      fMw(2,2)=-1;
      
      return fMw*wMg*gMk;
  
}


void Plane::changeFrame(const Eigen::Matrix4f & cMo)
{
  // Save current plane parameters
  float  Ao = this->parameters(0), 
         Bo = this->parameters(1), 
         Co = this->parameters(2), 
         Do = this->parameters(3);
         
  this->parameters(0) = cMo(0,0)*Ao + cMo(0,1)*Bo + cMo(0,2)*Co;
  this->parameters(1) = cMo(1,0)*Ao + cMo(1,1)*Bo + cMo(1,2)*Co;
  this->parameters(2) = cMo(2,0)*Ao + cMo(2,1)*Bo + cMo(2,2)*Co;
  this->parameters(3) = Do - (cMo(0,3)*this->parameters(0) + 
                              cMo(1,3)*this->parameters(1) + 
                              cMo(2,3)*this->parameters(2));
  
  // normalise
  this->parameters/=(-this->parameters(3));
                              
}
  
}
