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
#include <libgaitan/encoder.h>
#include <libgaitan/synchro.h>
#include <libgaitan/foot.h>
#include <libgaitan/geometry.h>


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

#include <visp/vpConfig.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpConfig.h>
#include <visp/vpPlot.h>


using namespace gaitan;





void userInput (int argc, char** argv, std::string& path, std::string &pattern, int & nbImBegin,int & sampleStep, int & nbImEnd)
{
 
  // get the path name
	if (argc>1){
		path = argv[1];
	}
	else {
//    path ="/home/dune/Documents/data/kinect/essai4/";
      path ="/home/dune/Documents/data/encoder-kinect/essai1";
  }
  
  // get the number of images to treat
  if (argc>2){
    nbImBegin = atoi(argv[2]);
    }
  else  nbImBegin = 0;  
  
  if (argc>3){
    nbImEnd = atoi(argv[3]);
    }
  else  nbImEnd = nbImBegin+100;  
  
  if (argc>4){
    sampleStep = atoi(argv[4]);
    
    }
  else sampleStep=floor((nbImEnd-nbImBegin)/50);  
    
  
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
  
  

int pointCloud(Kinect*kinect, std::string path, 
                int nbIm, double distThreshold,
                Eigen::MatrixXf &pt, Eigen::MatrixXf &ptOut)
{
    kinect->pointCloud(path,nbIm,pt);
      
    std::cout << " nb point before the FOV " << pt.rows()<< std::endl;

    kinect->limitFov(pt, ptOut,distThreshold);
    
    std::cout << " nb point after the FOV " << pt.rows()<< std::endl;
    std::cout << " nb point rejected after the FOV " << ptOut.rows()<< std::endl;

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
  int nbIm, nbSampleBegin, nbSampleEnd, sampleStep;
  userInput(argc, argv, path, pattern, nbSampleBegin, sampleStep,nbSampleEnd);
  
  double distThreshold(0.02);    // min point-to-plane distance when removing points belonging to ground plane

  
  //-------------Create the kinect from path --------------------------------------//
  Kinect * kinect= new Kinect();  
    
  // ------------------- Visualisation Settings-------------------------------//
  Plane ground;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();  
 
    
  // ---------------------Loop----------------------------------------//
   bool isFirst = true;
   int iteration(0);
  
   for(int index=nbSampleBegin ; index<=nbSampleEnd ; index+=sampleStep ){
    // prompt message
    std::cout << "----------------------" << std::endl ;
    std::cout << "  Iteration "           << iteration << std::endl ;
    std::cout << "----------------------" << std::endl ;
    iteration++;
         
         
    // create the point cloud as an eigen matrix                    
    Eigen::MatrixXf pt,ptOut;
    pointCloud(kinect,path,index,distThreshold,pt,ptOut);

    // Display
         
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Conversion::convert(pt,colorCloud, 0, 255,0);
    Conversion::convert(ptOut,colorCloud, 255, 0,0);
    
    // viewer remove all points, ok even if there is no point inside
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    
    // add the points in the viewer
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colorCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud, rgb,"sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); 
  
  
    viewer->spinOnce ();
  }
  
  
  return 0;
      
    
   }



