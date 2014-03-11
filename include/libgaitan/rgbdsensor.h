#ifndef GAITAN_RGBDSENSOR_H
#define GAITAN_RGBDSENSOR_H

#include <libgaitan/sensor.h>
#include <libgaitan/table.h>
#include <libgaitan/plane.h>
#include <libgaitan/box.h>

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
     std::string confFilename ; /*! path to the conf file*/
     
     
    public :
     // invalid zone made of boxes
     std::vector<Box> forbiddenZone;
     Box fov ; // authorized field of view 
    
    
    public:	
    
     // constructor and destructor
     RGBDSensor();
     RGBDSensor(double fx, double fy, double cx, double cy) ;
     virtual ~RGBDSensor();
     
     
     // accessors
     inline double getFx(){return this->fx;}
     inline double getFy(){return this->fy;}
     inline double getCx(){return this->cx;}
     inline double getCy(){return this->cy;}
      
     // modifiers
     inline void setFov(const  Box & fieldOfView){this->fov=fieldOfView;}
    
     // sensor class methods
     void init();
     void print();
     void print(const int & beg, const int & end);
     //config
     int saveConfFile(const std::string &pathName);
     
     //config
     int loadConfFile(const std::string &pathName);
     
     //time
     int loadTimeSampling(const std::string &pathName);
     inline float timestamp(const int & iter){return this->data->data(iter,1);}
     
     
     
     // rgbd sensor method
     void setRgbFilenamePattern(const std::string &s){this->rgbFilenamePattern=s;}
     void setDepthFilenamePattern(const std::string &s){this->depthFilenamePattern=s;}
     void setTimeFilename(const std::string &s){this->timeFilename=s;}
     void setConfFilename(const std::string &s){this->confFilename=s;}
     
    
    
    /*!
     * \brief Set the internal parameters  
     */
     void setIntrinsicParameters(double fx, double fy, double cx, double cy);
     
    
    
    /*! \brief compute the transformation between the kinect and the ground frame
     *  \param is an eigen matrix
     */
    Plane extrinsicCalibration(const Eigen::MatrixXf& pointCloud,double confidence=0.02,
                 float lsize=0.01);
                 
    /*! \brief compute the transformation between the kinect and the ground frame
     *  \param is a PCL point Cloud
     */
    Plane extrinsicCalibration(const pcl::PointCloud<pcl::PointXYZ>::Ptr& simpleCloud,double confidence=0.02,
                 float lsize=0.01);


    /*! \brief create the depth filename using the num of the image and the folder pathname*/
    std::string depthPath(const std::string &pathName, const int& imNb);
    /*! \brief create the rgb filename using the num of the image and the folder pathname*/
    std::string rgbPath(const std::string &pathName, const int& imNb);
    /*! \brief create the time filename using the folder pathname*/
    std::string timePath(const std::string &pathName);
    /*! \brief create the conf filename using the folder pathname*/
    std::string confPath(const std::string &pathName);
    
    /*! \brief create the depth filename using the num of the image and the folder pathname*/
    Plane ground(const Eigen::MatrixXf& pointCloud,  double confidence=0.02);
    Plane ground(const pcl::PointCloud<pcl::PointXYZ>::Ptr& simpleCloud,  double confidence=0.02);
    
    Eigen::Matrix4f computePose(const Eigen::MatrixXf& pointCloud, double confidence=0.02); // deprecated
    int computePose(const Eigen::MatrixXf& pointCloud, Eigen::MatrixXf & output, double confidence=0.02);
        
        
    /*!segment the point cloud into sub point cloud*/    
    int segment( const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloudFilterd,
                 std::vector<pcl::PointIndices>& clusterIndices,
                 int minClusterSize, int maxClusterSize, double tol);
  
    /*! segment the point cloud into sub point cloud (do a filtering first)*/
    int segment( const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudFiltered,
                 std::vector<pcl::PointIndices>& clusterIndices,
                 int minClusterSize, 
                 int maxClusterSize, 
                 double tol, 
                 float lsize=0.01);
    
    /*! filter the point cloud*/
    int filter (       const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                       pcl::PointCloud<pcl::PointXYZ>::Ptr& cloudFiltered, float lsize=0.01);
                       
    
    /*! Change the frame where the points are expressed*/                   
    Eigen::MatrixXf changeFrame(const Eigen::MatrixXf & kP,const Eigen::Matrix4f & wMk); // deprecated
    /*! use this function better*/
    void changeFrame(const Eigen::MatrixXf & kP, Eigen::MatrixXf & outM,const Eigen::Matrix4f & wMk);
    /*! Change the frame where the points are expressed*/                   
    Eigen::MatrixXf changeFrame(const Eigen::MatrixXf & kP);// deprecated
    void changeFrame(const Eigen::MatrixXf & kP,Eigen::MatrixXf & outM);
                   

    int detectClusters(const Eigen::MatrixXf& pointCloud, 
                             pcl::PointCloud<pcl::PointXYZ>::Ptr&  cloudFeetFiltered,
                             std::vector<pcl::PointIndices>& clusterIndices,
                             float clusterTolerance=0.04, 
                             int minClusterSize=100,
                             int maxClusterSize=50000,
                             float leafSize=0.01);
                             
    int detectClusters(const Eigen::MatrixXf& pointCloud, 
                            pcl::PointCloud<pcl::PointXYZ>::Ptr&  cloudFeetFiltered,
                            std::vector<pcl::PointIndices>& clusterIndices,
                            float clusterTolerance=0.04, 
                            int minClusterSize=100,
                            int maxClusterSize=50000,
                            double planeDistThreshold=0.02, 
                            float leafSize=0.01);                         
                             
    
    /*! remove the points that are on the ground ie z=0 */
    int clearGroundPoints( Eigen::MatrixXf & ptsIn, Eigen::MatrixXf & ptsOut, double distThreshold, Plane* gplane=new Plane(0,0,1,0))   ;

    /*! remove the points that are in the forbidden area */ 
    int clearForbiddenZone( Eigen::MatrixXf & ptsIn, Eigen::MatrixXf & ptsOut, double distThreshold ); 

    /*! remove the points that are out of the field of view */
    int limitFov(Eigen::MatrixXf & ptsIn, Eigen::MatrixXf & ptsOut, double & distThreshold);

    /*! init the list of boxes where the points have to be removed Valid only in ground frame*/
    int initForbiddenBoxes(const Eigen::MatrixXf & pts,
                          float clusterTolerance=0.04, 
                          int minClusterSize=150,
                          int maxClusterSize=10000,
                          float leafSize=0.01);
    
    int clusterBoundingBoxes(const Eigen::MatrixXf & pts, 
                             const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloudFeetFiltered,
                             const std::vector<pcl::PointIndices> &clusterIndices,
                             std::vector<Box> &boxes
                             );
    
    // to be implemented in daughter class
    virtual int display() = 0 ;
    virtual int acquire(const std::string &path , bool flagDisp=true) = 0;
    virtual Eigen::MatrixXf pointCloud(const std::string & path, const int &index) = 0;
    
    
    protected:
    std::string path(std::string pathName, std::string filename, const int &index);
    
  };
}
#endif // kinect_H
