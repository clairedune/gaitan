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
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>

namespace gaitan
{
  
  /*!
   * \brief constructor of class RGBDSensor with default values 
   */
  RGBDSensor::RGBDSensor() : Sensor(),
      rgbFilenamePattern("RGB_%07d.ppm"),
      depthFilenamePattern("depth_%07d.pfm"),
      timeFilename("TimeSampling.dat"),
      confFilename("conf.dat"),
      gMk(Eigen::MatrixXf::Identity(4,4)), 
      fx(), fy(), cx(), cy(),fov(-0.6,0.2,-0.5,0.6,0.0,1.0){ 
        
      this->data  = new Table(2,10);
      // to remove all the artifacts under the ground
      Box underground (-10,10, -10, 10, -10,0);  
      forbiddenZone.push_back(underground);
    
        
      }
      
   /*! remove the points that are not in the field of view*/
  int RGBDSensor::limitFov(Eigen::MatrixXf & ptsIn, Eigen::MatrixXf & ptsOut, double & distThreshold)
  {

   this->fov.inlierSelection( ptsIn, 
                              ptsOut, 
                              distThreshold);
   return 1;
       
   }      
  
  /*!
   * \brief constructor of class RGBDSensor with default values 
   *  and given value for internal parameters
   * 
   */
   RGBDSensor::RGBDSensor(double fx, double fy, double cx, double cy) 
  : Sensor(),rgbFilenamePattern("RGB_%07d.ppm"),
    depthFilenamePattern("depth_%07d.pfm"),
    timeFilename("TimeSampling.dat"),
    confFilename("conf.dat"),
    gMk(Eigen::MatrixXf::Identity(4,4)),
    fov(-0.6,0.2,-0.5,0.6,0.0,1.0) 
  {  
         this->data  = new Table(2,10);

   // to remove all the artifacts under the ground
      Box underground (-10,10, -10, 10, -10,0);  
      forbiddenZone.push_back(underground);
   
   this->fx = fx ; 
   this->fy = fy ; 
   this->cx = cx ;  
   this->cy = cy ;
   return;
  }
  
  
  /*!
   * \brief destructor of the RGBDSensor Class 
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
    
    for (std::vector<Box>::iterator it = this->forbiddenZone.begin() ; it != this->forbiddenZone.end(); ++it)
      {
        (*it).print();
      }    
    
  }
  
  void RGBDSensor::print(const int & deb , const int& end)
  {
    std::cout << "RGBD Sensor set up" << std::endl;
    std::cout << "internal parameters :\n"
         << "fx = " << this->fx << "\t"
         << "fy = " << this->fy << "\t"
         << "cx = " << this->cx << "\t"
         << "cy = " << this->cy << std::endl;    
    std::cout << "external parameters : \n";
    std::cout << this->gMk << std::endl;  
    
    for (std::vector<Box>::iterator it = this->forbiddenZone.begin() ; it != this->forbiddenZone.end(); ++it)
      {
        (*it).print();
      }    
    
    std::cout << "Time sampling  and image number : "<< std::endl;
    this->data->print(deb, end);
    
  }
  
  
  
  
  
  
/*! Write a conf file with the sensor param and the authorized areas */
int RGBDSensor::saveConfFile(const std::string &pathName)
  {
  
    std::string filename = confPath(pathName);
    std::ofstream file(filename.c_str(), std::ios::out);

	  if(file){
	  			file << this->fx << "\t" << this->fy << "\t" << this->cx << "\t" << this->cy << std::endl; 
          file << this->gMk << std::endl;
          file<< fov.getMinX() << "\t"<<fov.getMaxX()<< std::endl;
          file<< fov.getMinY() << "\t"<<fov.getMaxY()<< std::endl;
          file<< fov.getMinZ() << "\t"<<fov.getMaxZ()<< std::endl;
          file << this->forbiddenZone.size()<< std::endl;
          for (std::vector<Box>::iterator it = this->forbiddenZone.begin() ; it != forbiddenZone.end(); ++it)
          {
            file<< (*it).getMinX() << "\t"<<(*it).getMaxX()<< std::endl;
            file<< (*it).getMinY() << "\t"<<(*it).getMaxY()<< std::endl;
            file<< (*it).getMinZ() << "\t"<<(*it).getMaxZ()<< std::endl;
          }
      		file.close();
    	}
    else 
    {
      std::cerr << "impossible d'ouvrir le fichier " << this->confFilename<<std::endl;
      return 0;
    }
     return 1;  
  }
  
  
/*! Read the time file and set the data */
int RGBDSensor::loadTimeSampling(const std::string &pathName)
{
   std::string filename = this->timePath(pathName); 
   return this->load(filename);
}  

/*! Read the conf file to set the fobidden area value */
int RGBDSensor::loadConfFile(const std::string &pathName)
{
    std::string filename = confPath(pathName);
    //std::ofstream file(filename.c_str(), std::ios::out);
    std::ifstream file(filename.c_str(), std::ios::in);
    
    forbiddenZone.clear();
    
    
	  if(file){
	  			file >> this->fx ;
          file >> this->fy ; 
          file >> this->cx ;
          file >> this->cy ; 
          
          for(int i=0;i<4;i++)
            for(int j=0;j<4;j++){
              file >> this->gMk(i,j);
            }
              
          float minX, minY, minZ, maxX, maxY, maxZ;   
          file >> minX;
          file >> maxX;
          file >> minY;
          file >> maxY;
          file >> minZ;
          file >> maxZ;
          fov.setParameters(minX, maxX, minY, maxY, minZ, maxZ); 
           
          int nbBox;
          file >> nbBox;
          //std::cout << "NB BOX------>" << nbBox << std::endl ;
          for(int i=0; i<nbBox ; i++)
          {
            
           
           file >> minX;
           file >> maxX;
           file >> minY;
           file >> maxY;
           file >> minZ;
           file >> maxZ;
            
           Box tmp(minX, maxX, minY, maxY, minZ, maxZ);
           forbiddenZone.push_back(tmp); 
            
          } 
      		file.close();
    	}
    else 
    {
      std::cerr << "impossible d'ouvrir le fichier " << this->confFilename<<std::endl;
      return 0;
    }
     return 1;  
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
std::string RGBDSensor::depthPath(const std::string & pathName, const int& index){
    return this->path(pathName,this->depthFilenamePattern, index);  
    }
    
std::string RGBDSensor::rgbPath(const std::string &pathName, const int& index){
    return this->path(pathName,this->rgbFilenamePattern, index);  
    }
    
std::string RGBDSensor::timePath(const std::string & pathName){
    std::string tmp = pathName+"/"+this->timeFilename;
    return tmp;   
    }
    
std::string RGBDSensor::confPath(const std::string & pathName){
    std::string tmp = pathName+"/"+this->confFilename;
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
  //pcl::PointCloud<pcl::PointXYZ>::Ptr simpleCloud(new pcl::PointCloud<pcl::PointXYZ>);
  //Conversion::convert(pointCloud,simpleCloud);
  //plane = this->ground(simpleCloud,confidence);
  
  
  // OR Our method
  plane.findParameters(ptsIn,ptsOut,confidence);
  
  
  return plane;
    
}    
  
/*!
 * 
 * \brief detect the ground plane    er
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
   
      
      // Create the filtering object
     // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
     // sor.setInputCloud (cloud);
     // sor.setMeanK (50);
     // sor.setStddevMulThresh (1.0);
     // sor.filter (*cloudFiltered);
      
       // Create the filtering object
 //pcl::PassThrough<pcl::PointXYZ> pass;
 // pass.setInputCloud (cloud);
  //pass.setFilterFieldName ("z");
  //pass.setFilterLimits (0.0, 1.0);
  //pass.setFilterLimitsNegative (true);
  //pass.filter (*cloudFiltered);
      
      
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

Plane RGBDSensor::extrinsicCalibration(const Eigen::MatrixXf& pointCloud,double confidence,
                 float lsize)
{
      // create a PCL point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr simpleCloud(new pcl::PointCloud<pcl::PointXYZ>);
      Conversion::convert(pointCloud,simpleCloud);
      return this->extrinsicCalibration(simpleCloud,confidence,lsize);
      //Plane plane = this->ground(pointCloud,  confidence);
      // align the point cloud with the ground
      //this->gMk = plane.computeTransformation(); 
      //return 1;
} 

Plane RGBDSensor::extrinsicCalibration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& simpleCloud,double confidence,
                 float lsize)
{
      
      //filtering the simple cloud every 1cm
      pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
      this->filter( simpleCloud,filteredCloud, lsize);
      
      // compute the plane on this filteredcloud
      Plane plane = this->ground(filteredCloud,  confidence);
      //plane.print();
      // align the point cloud with the ground
      this->gMk = plane.computeTransformation(); 
       
      return plane;
} 

/*!
 * Detect clusters 
 * 
 */
int RGBDSensor::detectClusters(const Eigen::MatrixXf& pointCloud, 
                               pcl::PointCloud<pcl::PointXYZ>::Ptr&  cloudFeetFiltered,
                               std::vector<pcl::PointIndices>& clusterIndices,
                               float clusterTolerance, 
                               int minClusterSize,
                               int maxClusterSize,
                               float leafSize)
{
  // convert these point to a pcl point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeet(new pcl::PointCloud<pcl::PointXYZ>);
  Conversion::convert(pointCloud,cloudFeet);
  
  // filter the point of cloud to get regular grid of leafSize
  this->filter(cloudFeet,cloudFeetFiltered, leafSize);

  // segment the point filtered point cloud
  this->segment(cloudFeetFiltered,clusterIndices,minClusterSize, maxClusterSize, clusterTolerance);
  
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
      Eigen::MatrixXf ptsIn(pointCloud), ptsOut(1,3) ;
      plane.inlierSelection(ptsIn, ptsOut, planeDistThreshold);
      
      // align the point cloud with the ground
      this->gMk = plane.computeTransformation();
      Eigen::MatrixXf gPtsOut = this->changeFrame(ptsOut);
     
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
 
int RGBDSensor::clearGroundPoints(Eigen::MatrixXf & ptsIn,Eigen::MatrixXf & ptsOut,double distThreshold, Plane * gplane)   
{
  // select the points that are not on the ground
  //Plane gplane(0,0,1,0);
  //gplane->print();
  gplane->inlierSelection(ptsIn, ptsOut, distThreshold);  
  return 1;    
}


/*!
 *
 * \brief Set boxes around clusters
 * 
 *   \warning the point coordinates should be expressed in the Ground frame
 * 
 */ 
int RGBDSensor::clusterBoundingBoxes(const Eigen::MatrixXf & pts, 
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFeetFiltered,
                             const std::vector<pcl::PointIndices> &clusterIndices,
                             std::vector<Box> &boxes
                             )
{ 
 
  int j=0;
  // Creating the Clusters
  for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
         cloudCluster->points.push_back (cloudFeetFiltered->points[*pit]); 
         cloudCluster->width = cloudCluster->points.size ();
         cloudCluster->height = 1;
         cloudCluster->is_dense = true;
        
        // create a Matrix cluster
        Eigen::MatrixXf ptsCluster;
        Conversion::convert(cloudCluster, ptsCluster);                    
        Box box;
        box.findParameters(ptsCluster);
        boxes.push_back(box); 
//        std::cout << "Point cloud in the forbidden area: " << cloudCluster->points.size () << " data points." << std::endl;
        j++;
    }
  return 1;
}
/*!
 *
 * \brief Use a depthmap to find the clusters of points that are due to the wheels and
 * artifacts
 * 
 *  \warning the point cloud should be taken without the foot of the person.
 *  \warning the point coordinates should be expressed in the Ground frame
 * 
 */ 
int RGBDSensor::initForbiddenBoxes(const Eigen::MatrixXf & pts, 
                               float clusterTolerance, 
                               int minClusterSize,
                               int maxClusterSize,
                               float leafSize)
{
  // 1. convert the point cloud to a PCL point cloud, 
  // 2. filter 
  // 3. segment it  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeetFiltered (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointIndices> clusterIndices;
  this->detectClusters(pts, cloudFeetFiltered,clusterIndices, clusterTolerance, minClusterSize,maxClusterSize,leafSize);
  this->clusterBoundingBoxes(pts, cloudFeetFiltered,clusterIndices,this->forbiddenZone);
  return 1;
}

     
/*! remove the points that are in the forbidden area */ 
int RGBDSensor::clearForbiddenZone( Eigen::MatrixXf & ptsIn, Eigen::MatrixXf & ptsOut, double distThreshold ){
      return Box::inlierSelection( this->forbiddenZone,ptsIn,ptsOut,distThreshold);
}
  
  
  
}
