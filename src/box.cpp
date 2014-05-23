
#include <libgaitan/box.h>
#include <Eigen/Dense>
#include <iostream>
#include <vector>
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
     
     
    /*! Compute the distance between two boxes*/
    float Box::distance(Box & box1,Box & box2)
    {
      
      float distanceX(std::min(abs(box1.getMinX()-box2.getMaxX()),abs(box2.getMinX()-box1.getMaxX())));
      float distanceY(std::min(abs(box1.getMinY()-box2.getMaxY()),abs(box2.getMinY()-box1.getMaxY())));
      float distanceZ(std::min(abs(box1.getMinZ()-box2.getMaxZ()),abs(box2.getMinZ()-box1.getMaxZ())));
      

      return std::min(distanceX, std::min(distanceY, distanceZ));  
    } 
     
     
   //  Box & Box::operator= (Box  box){
   //       this->parameters = box.getParameters();
   //       return *this;    
   //  }  
     
    void Box::print(){
		 std::cout <<"The box parameters are:\n" << 
      "X min :" << this->parameters(0) << " \t X max :" << this->parameters(1)<< std::endl<<
      "Y min :" << this->parameters(2) << " \t Y max :" << this->parameters(3)<< std::endl<<
      "Z min :" << this->parameters(4) << " \t Y max :" << this->parameters(5)<< std::endl<<
      "X center : " <<this->getX() << "\t "<<
      "Y center : " <<this->getY() <<  "\t "<<
      "Z center : " <<this->getZ() <<  std::endl;
		 }
		 
     
 /*!
  * 
  * \brief test if a point P(x, y,z) is in the box
  * \return true if the point is in the box, else returns false  
  */    
 bool Box::isInBox(float x, float y, float z, float distThreshold){
   
 if(     (x > this->parameters(0) - distThreshold) && (x < this->parameters(1) + distThreshold)
          && (y > this->parameters(2) - distThreshold) && (y < this->parameters(3) + distThreshold)
          && (z > this->parameters(4) - distThreshold) && (z < this->parameters(5) + distThreshold)){
         return true;
        }
      else{
          return false;
        }  
 }    
     
     
   /*! 
    * \brief Select the point that are in the Box
    * Divide the set of point into two parts : the inliers and the outliers
    * the ptsIn matrix contains all the points. 
    * the ptsOut matrix contains no point at the beginning
    */
int Box::inlierSelection( std::vector<Box>& boxes,
                          Eigen::MatrixXf & ptsIn, 
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
    bool isInside;
    
    for(int i=0; i<max; i++){
      
      isInside = false;
      float x = pts(i,0);
      float y = pts(i,1);
      float z = pts(i,2);
    
      for (std::vector<Box>::iterator it = boxes.begin() ; it != boxes.end(); ++it)
      {
        if((*it).isInBox(x,y,z,distThreshold)) isInside=true;
      }
      
      if( isInside   ){
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
            
    ptsIn.conservativeResize(indexIn,3); // the matrix will be smaller that max 
    ptsOut.conservativeResize(indexOut,3); // the matrix will be smaller that max 
    
        
        return 1;

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
      if( this->isInBox(x,y,z,distThreshold)   ){
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
            
    ptsIn.conservativeResize(indexIn,3); // the matrix will be smaller that max 
    ptsOut.conservativeResize(indexOut,3); // the matrix will be smaller that max 
    
        
        return 1;

}

    /*!
     * Find the box around point cloud 
     */
     int Box::findParameters(const MatrixXf & pts ){
       float minx(300), maxx(-300); 
       float miny(300), maxy(-300); 
       float minz(300), maxz(-300);
       float x(0.0), y(0.0),z(0.0); 
       
       for(int i=0 ; i< pts.rows() ; i++)
       {
            x = pts(i,0);
            y = pts(i,1);
            z = pts(i,2);
            
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
     
     //int Box::findParameters()
     //{
       
     //}
     
     
    
     VectorXf Box::computeDistance(const MatrixXf & pts){/*to be implemented */ return MatrixXf::Zero(1,1);}
     float Box::computeDistance(const double &X, const double &Y, const double & Z) {/*to be implemented*/ return 0.0f;}


   
}
