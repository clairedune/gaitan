
#include <libgaitan/box.h>
#include<Eigen/Dense>
#include<iostream>

#include <pcl/common/common_headers.h>

   
using namespace Eigen;

namespace gaitan
{
     Box::Box() : Shape(MatrixXf::Zero(6,1)){
		 }
		 
     Box::Box(const double& minx, const double& maxx, 
                             const double& miny, const double& maxy,
                             const double& minz, const double& maxz){
                               
         this->setParameters(minx, maxx, 
                             miny, maxy,
                             minz, maxz);
       
     }
    
    
     Box::Box(const VectorXf& param) : Shape(param){
     }
    
     void Box::setParameters(const double& minx, const double& maxx, 
                             const double& miny, const double& maxy,
                             const double& minz, const double& maxz)
    {
         this->parameters.resize(6);
         this->parameters(0) =  minx ;
         this->parameters(1) =  maxx ;
         this->parameters(2) =  miny ;
         this->parameters(3) =  maxy ;
         this->parameters(4) =  minz ;
         this->parameters(5) =  maxz ;
    }
    
         
     
     
     Box::~Box(){}
     
     
     double Box::getMinX(){
       return this->parameters(0);
       }
     double Box::getMaxX(){
       return this->parameters(1);
       }
     double Box::getMinY(){
       return this->parameters(2);
       }
     double Box::getMaxY(){
       return this->parameters(3);
       }
     double Box::getMinZ(){
       return this->parameters(4);
       }
     double Box::getMaxZ(){
       return this->parameters(5);
       }
     
     
   //  Box & Box::operator= (Box  box){
   //       this->parameters = box.getParameters();
   //       return *this;    
   //  }  
     
    void Box::print(){
		 std::cout <<"The box parameters are:\n" << this->parameters << std::endl;
		 }
		 
   /*! 
    * \brief Select the point that are in the Box
    * Divide the set of point into two parts : the inliers and the outliers
    * the ptsIn matrix contains all the points. 
    * the ptsOut matrix contains no point at the beginning
    */
   
int Box::inlierSelection( Eigen::MatrixXf & ptsIn, 
                          Eigen::MatrixXf & ptsOut, 
                           double & distThreshold){
   
    // store the current data in a temp matrix
    Eigen::MatrixXf pts(ptsIn);
    
        
    // resize the input matrixes in and out  
    int max(pts.rows()); // nb points
    ptsIn.resize(max,3); // the matrix will be smaller that max 
    ptsOut.resize(max,3); // the matrix will be smaller that max 

    
    // select the point in and out
    int indexIn(0), indexOut(0);
    for(int i=0; i<max; i++){
      
      float x = pts(i,0);
      float y = pts(i,1);
      float z = pts(i,2);
      
      
      
      // if x > minX - distThreshold && x < maxX + distThreshold 
      // AND if y > minY - distThreshold && y < maxY + distThreshold 
      // AND if z > minZ - distThreshold && z < maxZ + distThreshold 
      if(    (x > this->parameters(0) - distThreshold) && (x < this->parameters(1) + distThreshold)
          && (y > this->parameters(2) - distThreshold) && (y < this->parameters(3) + distThreshold)
          && (z > this->parameters(4) - distThreshold) && (z < this->parameters(5) + distThreshold)){
          ptsIn(indexIn,0) = x;
          ptsIn(indexIn,1) = y;
          ptsIn(indexIn,2) = z; 
          indexIn++;
        }
      else{
          ptsOut(indexOut,0) = x;
          ptsOut(indexOut,1) = y;
          ptsOut(indexOut,2) = z; 
          indexOut++;
        }
      }
    std::cout << " test2" << std::endl;
            
    ptsIn.conservativeResize(indexIn,3); // the matrix will be smaller that max 
    ptsOut.conservativeResize(indexOut,3); // the matrix will be smaller that max 
    
        std::cout << " test3" << std::endl;
        
        return 1;

}

    /*!
     * Find the box around point cloud 
     */
     int Box::findParameters(const MatrixXf & pts ){
       float minx(300), maxx(-300); 
       float miny(300), maxy(-300); 
       float minz(300), maxz(-300); 
       
       for(int i=0 ; i< pts.rows() ; i++)
       {
            float x = pts(i,0);
            float y = pts(i,1);
            float z = pts(i,2);
            
            if (x<minx) minx=x;
            else if (x>maxx) maxx=x;  
 
            if (y<miny) miny=y;
            else if (y>maxy) maxy=y;  
 
            if (z<minz) minz=z;
            else if (z>maxz) maxz=z;  
       } 
        
       this->setParameters(minx, maxx, miny, maxy, minz, maxz);
            
       return 0;
     }
    
     VectorXf Box::computeDistance(const MatrixXf & pts){/*to be implemented */ return MatrixXf::Zero(1,1);}
     float Box::computeDistance(const double &X, const double &Y, const double & Z) {/*to be implemented*/ return 0.0f;}


   
}
