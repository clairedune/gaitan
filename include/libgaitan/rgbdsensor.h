#ifndef GAITAN_RGBDSENSOR_H
#define GAITAN_RGBDSENSOR_H

#include <libgaitan/sensor.h>
#include <libgaitan/table.h>
#include <libgaitan/plane.h>

#include <Eigen/Dense>

#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>


namespace gaitan
{
/*!
 * RGBDSensor class for all RGBD devices (kinect, xtion, etc.)
 * 
 * 
 * 
 */ 

  class RGBDSensor:public Sensor
  {   
    
    // private parameters
    protected: 
     double fx ; /*! pixel/m ratio in x axis */
     double fy ; /*! pixel/m ratio in y axis */
     double cx ; /*! x coord of the optical center in pixels */
     double cy ; /*! y coord of the optical center in pixels */
     
     // pose of the sensor in a frame attached to the ground
     Eigen::Matrix4f gMk ; /*! pose of the sensor wrt the ground */
     
     // name of the filename for RGB data, depth data, and time data
     std::string rgbFilenamePattern ; /*! root name for rgb file */
     std::string depthFilenamePattern ; /*! root name for depth file*/
     std::string timeFilename ; /*! root name for time file*/
     
     
    
    public:	
    
     // constructor and destructor
     RGBDSensor();
     RGBDSensor(double fx, double fy, double cx, double cy) ;
     virtual ~RGBDSensor();
    
    
     // sensor class methods
     void init();
     void print();
    
    
    /*!
     * \brief Set the internal parameters  
     */
     void setIntrinsicParameters(double fx, double fy, double cx, double cy);
     
    
    
    /*! \brief compute the transformation between the kinect and the ground frame
     */
    int extrinsicCalibration(const Eigen::MatrixXf& pointCloud,double confidence=0.02,
                 float lsize=0.01);

    /*! \brief create the depth filename using the num of the image and the folder pathname
     */
    std::string depthPath(std::string pathName, const int& imNb);
    /*! \brief create the rgb filename using the num of the image and the folder pathname
     */
    std::string rgbPath(std::string pathName, const int& imNb);
    /*! \brief create the time filename using the folder pathname
     */
    std::string timePath(std::string pathName);
    
    /*! \brief create the depth filename using the num of the image and the folder pathname
     */
    Plane ground(const Eigen::MatrixXf& pointCloud,  double confidence=0.02);
    Plane ground(const pcl::PointCloud<pcl::PointXYZ>::Ptr& simpleCloud,  double confidence=0.02);
    
    Eigen::Matrix4f computePose(const Eigen::MatrixXf& pointCloud, double confidence=0.02);
        
    int segment( const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudFilterd,
                 std::vector<pcl::PointIndices>& clusterIndices,
                 int minClusterSize, int maxClusterSize, double tol);
    
    int segment( const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudFiltered,
                 std::vector<pcl::PointIndices>& clusterIndices,
                 int minClusterSize, 
                 int maxClusterSize, 
                 double tol, 
                 float lsize=0.01);
    
    int filter (       const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudFiltered, float lsize=0.01);
                       
    Eigen::MatrixXf changeFrame(const Eigen::MatrixXf & kP,const Eigen::Matrix4f & wMk);
    Eigen::MatrixXf changeFrame(const Eigen::MatrixXf & kP);
                   

    int detectClusters(const Eigen::MatrixXf& pointCloud, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr&  cloudFeetFiltered,
                             std::vector<pcl::PointIndices>& clusterIndices,
                                  float clusterTolerance=0.05, 
                                  int minClusterSize=100,
                                  int maxClusterSize=10000,
                                  double planeDistThreshold=0.02,  
                                  float leafSize=0.01);
    
    int removeGroundPoints(Eigen::MatrixXf ptsIn,Eigen::MatrixXf ptsOut,double distThreshold)   ;

    
    
    
    // to be implemented in daughter class
    virtual int display() = 0 ;
    virtual int acquire(const std::string &path , bool flagDisp=true) = 0;
    virtual Eigen::MatrixXf pointCloud(const std::string & path, const int &index) = 0;
    
    
    protected:
    std::string path(std::string pathName, std::string filename, const int &index);
    
  };
}
#endif // kinect_H
