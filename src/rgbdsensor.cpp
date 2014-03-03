/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// TODO : everything in pcl?


#include <libgaitan/rgbdsensor.h>
#include <libgaitan/conversion.h>
#include <libgaitan/plane.h>

#include <sys/time.h> // to get time
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <math.h>

#include <Eigen/Dense>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace gaitan
{
  
  /*!
   * \brief constructor of class RGBDSensor with default values 
   * 
   * 
   * 
   */
  RGBDSensor::RGBDSensor() : Sensor(),
      rgbFilenamePattern("RGB_%07d.ppm"),
      depthFilenamePattern("depth_%07d.pfm"),
      timeFilename("TimeSampling.dat"),
      gMk(Eigen::MatrixXf::Identity(4,4)), 
      fx(), fy(), cx(), cy(){ return;}
      
  
  /*!
   * \brief constructor of class RGBDSensor with default values 
   *  and given value for internal parameters
   * 
   * 
   */
   RGBDSensor::RGBDSensor(double fx, double fy, double cx, double cy) 
  : Sensor(),rgbFilenamePattern("RGB_%07d.ppm"),
    depthFilenamePattern("depth_%07d.pfm"),
    timeFilename("TimeSampling.dat"),
    gMk(Eigen::MatrixXf::Identity(4,4))  
  {  
   
   this->fx = fx ; 
   this->fy = fy ; 
   this->cx = cx ;  
   this->cy = cy ;
   return;
  }
  
  
  /*!
   * \brief destructor of the RGBDSensor Class 
   * 
   * 
   * 
   */
 RGBDSensor::~RGBDSensor()
  {
  
  }
  
  
   void RGBDSensor::setIntrinsicParameters(double fx, double fy, double cx, double cy)
   {
      this->fx = fx ; 
      this->fy = fy ; 
      this->cx = cx ;  
      this->cy = cy ;
    }
  
  
   /*! 
   * \brief display the parameters of the Sensor 
   * 
   * 
   * 
   */
  void RGBDSensor::print()
  {
    std::cout << "RGBD Sensor set up" << std::endl;
    std::cout << "internal parameters :\n"
         << "fx = " << this->fx << "\t"
         << "fy = " << this->fy << "\t"
         << "cx = " << this->cx << "\t"
         << "cy = " << this->cy << std::endl;    
    std::cout << "external parameters : \n";
    std::cout << this->gMk << std::endl;      
    
  }

  
/*! 
 * \brief create a full path using the num of the image and the folder pathname
 * \param pathName, a path name
 * \param filename, the pattern of the filename, should include a %d to include the right index
 * \param index, an interger corresponding to the sample number
*/
std::string RGBDSensor::path(std::string pathName, std::string filename, const int& index)
{
    std::string tmp = pathName+"/"+filename;
    char buf[100];
    sprintf(buf,tmp.c_str(),index);
    std::string tmp2(buf);      
    return tmp2; 
  }

/*! 
 * \brief create the depth filename using the num of the image and the folder pathname
 * \param pathName, a path name to the forlder containing the data
 * \param filename, the pattern of the filename, should include a %d to include the right index
 * \param index, an interger corresponding to the sample number
*/
std::string RGBDSensor::depthPath(std::string pathName, const int& index){
    return this->path(pathName,this->depthFilenamePattern, index);  
    }
    
std::string RGBDSensor::rgbPath(std::string pathName, const int& index){
    return this->path(pathName,this->rgbFilenamePattern, index);  
    }
    
std::string RGBDSensor::timePath(std::string pathName){
    std::string tmp = pathName+"/"+this->timeFilename;
    return tmp;   
    }
    
  
/*! 
 * \brief detect the ground plane and deduce RGBDSensor pose wrt ground    
 * assume that most of the points are on the ground
 */
Eigen::Matrix4f RGBDSensor::computePose(const Eigen::MatrixXf& pointCloud,  double confidence){
  Plane plane = this->ground(pointCloud,confidence);
  Eigen::Matrix4f gMk = plane.computeTransformation();
  return gMk;
  }    
    
    
/*! 
 * \brief convert the point cloud to a pcl point cloud and detect the ground plane    
 * assume that most of the points are on the ground
 */    
Plane RGBDSensor::ground(const Eigen::MatrixXf& pointCloud,  double confidence){

  // find the coeff of the main plane
  Eigen::MatrixXf ptsIn(pointCloud), ptsOut(3,1) ;
 
  // create the plane
  Plane plane;
  
  // convert the current point cloud into a pcl point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr simpleCloud(new pcl::PointCloud<pcl::PointXYZ>);
  Conversion::convert(pointCloud,simpleCloud);
  plane = this->ground(simpleCloud,confidence);
  
  
  // OR Our method
  //plane.findParameters(ptsIn,ptsOut,confidence);
  
  
  return plane;
    
}    
  
/*!
 * 
 * \brief detect the ground plane    
 * \warning assume that most of the points are on the ground
 * 
 */   
Plane RGBDSensor::ground(const pcl::PointCloud<pcl::PointXYZ>::Ptr& simpleCloud,  double confidence){
  Plane plane;
  // PCL method
  plane.findParameters(simpleCloud,confidence);
  return plane;  
}  
  
  
/*!
 * 
 * \brief Organise the point cloud as a kdtree, segment the point cloud
 * \warning Input is considered to be a filtered point cloud
 * 
 */ 
  
int RGBDSensor::segment(   const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudFiltered,
                           std::vector<pcl::PointIndices>& clusterIndices,
                           int minClusterSize, int maxClusterSize, double tol)
{
    // Creating the KdTree object for the search method of the extraction 
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloudFiltered);     
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (tol) ;
    ec.setMinClusterSize (minClusterSize);
    ec.setMaxClusterSize (maxClusterSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloudFiltered);
    ec.extract (clusterIndices);
    
    return 1; 
}
  
  
int RGBDSensor::segment( const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                     pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudFiltered,
                     std::vector<pcl::PointIndices>& clusterIndices,
                     int minClusterSize, int maxClusterSize, double tol, float lsize)
{
  
    this->filter (cloud,cloudFiltered,lsize) ; 
    return this->segment(cloudFiltered, clusterIndices, minClusterSize, maxClusterSize, tol);
    
    return 1; 
}  

int RGBDSensor::filter (       const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                           pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudFiltered, float lsize){
   
   // Create the filtering object: downsample the dataset using a leaf size of 1cm
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      vg.setInputCloud (cloud);
      vg.setLeafSize (lsize, lsize, lsize);
      vg.filter (*cloudFiltered);
     
   return 1;
  }
 
/*!
 * 
 * \brief Compute the transformation between the RGBDSensor and the ground
 * \warning Assume that the RGBDSensor is oriented towards the ground
 * \warning Assume that more point belong to the ground than to the object
 * 
 */  

int RGBDSensor::extrinsicCalibration(const Eigen::MatrixXf& pointCloud,double confidence,
                 float lsize)
{
      // create a PCL point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr simpleCloud(new pcl::PointCloud<pcl::PointXYZ>);
      Conversion::convert(pointCloud,simpleCloud);
      
      //filtering the simple cloud every 1cm
      pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
      this->filter( simpleCloud,filteredCloud, lsize);
      
      // compute the plane on this filteredcloud
      Plane plane = this->ground(filteredCloud,  confidence);
  
      // select the inliers
      Eigen::MatrixXf ptsIn(pointCloud), ptsOut(3,1) ;
      plane.inlierSelection(ptsIn, ptsOut, confidence);
      
      // align the point cloud with the ground
      this->gMk = plane.computeTransformation(); 
       
      return 1;
} 

/*!
 * Detect clusters over the ground in the ground plane
 * DEPRECATED TOO MUCH THINGS HIDDEN THERE
 */
int RGBDSensor::detectClusters(const Eigen::MatrixXf& pointCloud, 
                            pcl::PointCloud<pcl::PointXYZ>::Ptr&  cloudFeetFiltered,
                            std::vector<pcl::PointIndices>& clusterIndices,
                                  float clusterTolerance, 
                                  int minClusterSize,
                                  int maxClusterSize,
                                  double planeDistThreshold, 
                                  float leafSize)
{
      // create a PCL point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr simpleCloud(new pcl::PointCloud<pcl::PointXYZ>);
      Conversion::convert(pointCloud,simpleCloud);
      
      //filtering the simple cloud every 1cm
      pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
      this->filter( simpleCloud,filteredCloud, leafSize);
      
      // compute the plane on this filteredcloud
      Plane plane = this->ground(filteredCloud, planeDistThreshold);
  
      // select the inliers
      Eigen::MatrixXf ptsIn(pointCloud), ptsOut(3,1) ;
      plane.inlierSelection(ptsIn, ptsOut, planeDistThreshold);
      
      // align the point cloud with the ground
      Eigen::Matrix4f gMk = plane.computeTransformation();
      Eigen::MatrixXf gPtsOut = this->changeFrame(ptsOut,gMk);
     
      // keep only the point that are not on ground
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeet(new pcl::PointCloud<pcl::PointXYZ>);
      Conversion::convert(gPtsOut,cloudFeet);

      // segment the point that are not on the ground
      this->segment(cloudFeet, cloudFeetFiltered,clusterIndices,minClusterSize, maxClusterSize, clusterTolerance);
  
}  



/*!
 *  align the camera frame with the ground
 */
Eigen::MatrixXf RGBDSensor::changeFrame(const Eigen::MatrixXf & kP)
{
    return this->changeFrame(kP, this->gMk);
}

/*!
 *  change point frame 
 *  FIXME : may be to set as static...
 */
Eigen::MatrixXf RGBDSensor::changeFrame(const Eigen::MatrixXf & oP,const Eigen::Matrix4f & wMo)
{
   Eigen::MatrixXf wP(oP.rows(),oP.cols());
   for(int i = 0 ; i < oP.rows(); i++ ){
      // create an homogenous point from 3D coordinate
      Eigen::Vector4f oPi;
      oPi(0) = oP(i,0);
      oPi(1) = oP(i,1);
      oPi(2) = oP(i,2);
      oPi(3) = 1;
      // change frame
      Eigen::Vector4f wPi  = wMo*oPi;
      // insert in the matrix
      wP(i,0) = wPi(0);
      wP(i,1) = wPi(1);
      wP(i,2) = wPi(2);
   }
   return wP;
}


/*!
 * 
 * \brief Prune the points that are on the ground plane, ie on the plane of equation
 * z=0, ie plane(0,0,1,0)
 * 
 * 
 */ 
 
int RGBDSensor::removeGroundPoints(Eigen::MatrixXf ptsIn,Eigen::MatrixXf ptsOut,double distThreshold)   
{
  // select the points that are not on the ground
  Plane gplane(0,0,1,0);
  gplane.inlierSelection(ptsIn, ptsOut, distThreshold);  
  return 1;    
}

  
}
