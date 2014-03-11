#include <sys/time.h> // to get time
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <math.h>

#include <Eigen/Dense>

#include <libgaitan/plane.h>
#include <libgaitan/box.h>
#include <libgaitan/conversion.h>
#include <libgaitan/kinect.h>

#include <visp/vpImageIo.h>


#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/pcl_base.h>
#include <cfloat>
#include "pcl/common/impl/common.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

using namespace gaitan;

void userInput (int argc, char** argv, std::string& path, std::string &pattern, int & nbImBegin, int & nbImEnd)
{
  
  pattern = "depth_%07d.pfm";
 
  // get the path name
	if (argc>1){
		path = argv[1];
	}
	else {
    path ="/home/dune/Documents/data/kinect/essai4";
  }
  
  // get the number of images to treat
  if (argc>2){
    nbImBegin = atoi(argv[2]);
    }
  else  nbImBegin = 60;  
  
  if (argc>3){
    nbImEnd = atoi(argv[3]);
    }
  else  nbImEnd = nbImBegin;  
  
  
}


int cubeVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer, Box box, float r, float g, float b, std::string name){
  
        viewer->addCube (
      box.getMinX(),
      box.getMaxX(),
      box.getMinY(),
      box.getMaxY(),
      box.getMinZ(),
      box.getMaxZ(), r, g,b, name); 
      
      return 1;
  }

boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
float  a, float  b, float  c, float  d)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  //------------------------------------
  //-----Add shapes at cloud points-----
  //------------------------------------
  //viewer->addLine<pcl::PointXYZRGB> (cloud->points[0],
  //                                   cloud->points[cloud->size() - 1], "line");
  //viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (a);
  coeffs.values.push_back (b);
  coeffs.values.push_back (c);
  coeffs.values.push_back (d);
 viewer->addPlane (coeffs, "plane");

  return (viewer);
}

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{


 //------------- INIT -----------------------------------------//
  std::string path, pattern, fullPath;
  int nbIm, nbImBegin, nbImEnd;
  userInput(argc, argv, path, pattern, nbImBegin, nbImEnd);
     
   
  // ------------ SET UP ------------------------------------//
  
  // segmentation parameters
  // TODO : make a conf file
  float clusterTolerance (0.04); // min dist between two cluster
  int minClusterSize(150);       // min size of a cluster
  int maxClusterSize(50000);     // max size of a cluster
  double confidence(0.02);       // confidence for plane detection
  float leafSize(0.005);          // size of the grid a filtered point cloud
  double distThreshold(0.02);    // min point-to-plane distance when removing points belonging to ground plane

  //-------------------------------------------------------------//
        
//  double fx(525.0), fy(525.0), cx(319.05), cy(239.5);
  Kinect * kinect= new Kinect();  
  if(!kinect->loadConfFile(path))
  {
      std::cerr   <<"No configuration file found. You need to init the kinect first"  << std::endl;
      return -1;
  }
  
  if(!kinect->loadTimeSampling(path))
  {
      std::cerr   <<"No timeFile file found. You need to init the kinect first"  << std::endl;
      return -2;
  }
  
  
  Plane ground;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();  
  
  bool isFirst = true;
  int iteration  =0;

  Box feetZone(-0.8,0,-0.4, 0.4, 0, 0.6);


  for(int nbIm=nbImBegin;nbIm<=nbImEnd ; nbIm++){ 
 
    
    // prompt message
    std::cout << "----------------------" << std::endl ;
    std::cout << "  Iteration " << iteration << std::endl ;
    std::cout << "  Image " << nbIm << std::endl ;
    std::cout  << " Time " <<std::setprecision(15)<<kinect->data->data(nbIm,1)<< std::endl; 
    std::cout << "----------------------" << std::endl ;
    iteration++;
         
    // create the point cloud as an eigen matrix
   
    Eigen::MatrixXf pt;
    kinect->pointCloud(path,nbIm,pt);
    if(pt.rows()<1) 
    {
      std :: cerr << "There is no point in the point cloud"<< std::endl; 
      return -2;
    }   
    std::cout << pt.rows() << "\t Total points" << std::endl;
    std::cout << pt.rows() << "\t Total points in field of view" << std::endl;
    

      
    // init the kinect pose wrt the ground only for the first image
    if(isFirst)
      ground = kinect->extrinsicCalibration(pt,confidence,leafSize);   
     isFirst=false;
 
    // select the points that are not on the ground
    Eigen::MatrixXf ptsGround(pt),ptsFeetNWheels(1,3) ;
    kinect->clearGroundPoints(ptsGround,ptsFeetNWheels,distThreshold,&ground);  
    
    
    // change the points frame
    Eigen::MatrixXf gPtsFeetNWheels  = kinect->changeFrame(ptsFeetNWheels);     
    
    
    //bounding box for the feet and wheels
    Box bbFeetNWheels;
    bbFeetNWheels.findParameters(gPtsFeetNWheels);
    
    //limit the fov
    Eigen::MatrixXf ptsOut; 
    kinect->setFov(feetZone);
    kinect->limitFov(gPtsFeetNWheels,ptsOut,distThreshold);

    // bounding box for the fov
    //Box bbFov;
    //bbFov.findParameters(gPtsFeetNWheels);

  
    // Clear forbidden zone corresponding to the wheels
    Eigen::MatrixXf gPtsWheels(gPtsFeetNWheels), gPtsFeet(1,3); 
    kinect->clearForbiddenZone(gPtsWheels, gPtsFeet, distThreshold);
     
    // bounding box for the feet 
    Box bbFeet;
    bbFeet.findParameters(gPtsFeetNWheels);
    bbFeet.print();         
             
             
    //------------------------------------------------------------//
    //
    //  Detect the feet cluster and trace bounding boxes
    //
    //------------------------------------------------------------//     
    std::vector<Box> feetBoundingBoxes;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeetFiltered (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> clusterIndices;
    kinect->detectClusters(gPtsFeet, cloudFeetFiltered,clusterIndices, clusterTolerance, minClusterSize,maxClusterSize,leafSize);
    kinect->clusterBoundingBoxes(gPtsFeet, cloudFeetFiltered,clusterIndices,feetBoundingBoxes);
             
             
             
    //------------------------------------------------------------//
    //
    //  Display the point cloud and the bounding boxes
    //
    //------------------------------------------------------------//         
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Conversion::convert(gPtsFeet,colorCloud, 0, 255,0);
    
    std::cout << colorCloud->points.size() << "\t pcl colour point cloud" << std::endl;     
    // viewer remove all points, ok even if there is no point inside
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    
    // add the points in the viewer
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colorCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud, rgb,"sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); 
  
    // bounding boxes
   // cubeVis(viewer, bbFeetNWheels, 1.0, 1.0,1.0,"feet and wheels");
    cubeVis(viewer, bbFeet, 0.5, 1.0,0.5,"feet");
    cubeVis(viewer, feetZone, 1.0, 0.5,0.5,"fov");
  
  
    // display the boxes corresponding to the forbidden zone
    for (int i =0; i<kinect->forbiddenZone.size();i++) {
       std::string tmp = "cube-%d";
       char buf[100];
       sprintf(buf,tmp.c_str(),i);
       std::string cubeName(buf);      
       cubeVis(viewer, kinect->forbiddenZone[i], 1.0, 0.0,0.0,cubeName);
    }
    // display the boxes corresponding to the feet
    float x, y, z;
    
    for (int i =0; i<feetBoundingBoxes.size();i++) {
       feetBoundingBoxes[i].center (x,y,z);
       std::cout << " Box "<< i << " :" << std::endl;
       feetBoundingBoxes[i].print();
       std::cout << "x : "<< x << "\t" ;
       std::cout << "y : "<< y << "\t" ;
       std::cout << "z : "<< z << "\t" << std::endl;
       std::cout << "volume : " << feetBoundingBoxes[i].volume() << std::endl;
       
       std::string tmp = "foot-%d";
       char buf[100];
       sprintf(buf,tmp.c_str(),i);
       std::string footName(buf);      
       cubeVis(viewer, feetBoundingBoxes[i], 0.0, 1.0,0.0,footName);
    }
    
    int elapse(0);  
    //while (!viewer->wasStopped ())
    while (elapse<3)
      {
         viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (10000));
        elapse++;
      }
   }
}


