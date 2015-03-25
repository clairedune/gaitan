#include <visp/vpConfig.h>

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

#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GDI))
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpTime.h>
#include <visp/vpImageIo.h>
#include <visp/vpRGBa.h>

using namespace gaitan;

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
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



void userInput (int argc, char** argv, std::string& path, int & nbIm, float & clusterTolerance)
{
  
  
 
  // get the path name
	if (argc>1){
		path = argv[1];
	}
	else {
    path ="/home/dune/Documents/data/kinect/essai2";
  }
  
  // get the number of the image to use for the init
  if (argc>2){
    nbIm = atoi(argv[2]);
    }
  else nbIm = 0;  
  
  
  if(argc>3){
    clusterTolerance = atof(argv[3]);
    } 
  else clusterTolerance = 0.1;
  
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
  
  // ------------ SET UP ------------------------------------//
  // segmentation parameters
  // TODO : make a conf file
  float clusterTolerance (0.12);  // min dist between two cluster
  int minClusterSize(50);         // min size of a cluster
  int maxClusterSize(50000);       // max size of a cluster
  double confidence(0.02);         // confidence for plane detection
  float leafSize(0.005);           // size of the grid a filtered point cloud
  double distThreshold(0.01);      // min point-to-plane distance when removing points belonging to ground plane
 
 //------------- INIT -----------------------------------------//
  std::string path, fullPath, pattern;
  pattern = "depth_%07d.pfm";
  int nbIm;
  userInput(argc, argv, path, nbIm, clusterTolerance);
  
  //-------------------------------------------------------------//
  double fx(525.0), fy(525.0), cx(319.05), cy(239.5);
  Kinect * kinect= new Kinect(fx,fy,cx,cy);  
      
  // create the point cloud as an eigen matrix
  Eigen::MatrixXf pointCloud = kinect->pointCloud(path,nbIm);
      
  // init the kinect pose wrt the ground
  Plane ground = kinect->extrinsicCalibration(pointCloud,confidence,leafSize);    
  
  // select the points that are not on the ground
  Eigen::MatrixXf ptsFeetNWheels(1,3) ;
  kinect->clearGroundPoints(pointCloud,ptsFeetNWheels,distThreshold,&ground);  
  
  // change the points cloud frame using the orientation of the ground
  Eigen::MatrixXf gPtsFeetNWheels  = kinect->changeFrame(ptsFeetNWheels);     
  Eigen::MatrixXf gPointCloud  = kinect->changeFrame(pointCloud);     
  
  std :: cout << pointCloud.rows() << std :: endl;
  std :: cout << ptsFeetNWheels.rows() << std :: endl;
  std :: cout << gPtsFeetNWheels.rows() << std :: endl;
  
  leafSize = 0.01;
  //init the boxes
  kinect->initForbiddenBoxes(gPtsFeetNWheels,clusterTolerance, minClusterSize, maxClusterSize,leafSize);
  std :: cout << "number of fobidden boxes: " << kinect->forbiddenZone.size() << std::endl;
  kinect->saveConfFile(path);

  //display all the boxes
  for(int i=0 ; i< kinect->forbiddenZone.size() ; i++)
  {
      std::cout << "----- "<< i << " ----- " << endl; 
      kinect->forbiddenZone[i].print();  
  }
  
      
      Eigen::MatrixXf gPtsWheels(gPtsFeetNWheels), gPtsFeet(1,3); 
      kinect->clearForbiddenZone(gPtsWheels, gPtsFeet, distThreshold);
             
      // viewer

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      Conversion::convert(gPtsWheels,colorCloud, 0, 0,255);
      Conversion::convert(gPointCloud,colorCloud, 255, 0,0);
      Conversion::convert(gPtsFeet,colorCloud, 0, 255,0);

      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      viewer = shapesVis(colorCloud,0.0f, 0.0f, 1.0f, 0.0f);
      
      
      if (kinect->forbiddenZone.size()>=1) 
        cubeVis(viewer, kinect->forbiddenZone[0], 1.0, 1.0,1.0, "underground");
      
      if (kinect->forbiddenZone.size()>=2)  
        cubeVis(viewer, kinect->forbiddenZone[1], 1.0, 0.0,0.0, "LF");
      
      if (kinect->forbiddenZone.size()>=3) 
        cubeVis(viewer, kinect->forbiddenZone[2], 0.0, 1.0,0.0, "RF");
      
      if (kinect->forbiddenZone.size()>=4) 
        cubeVis(viewer, kinect->forbiddenZone[3], 0.0, 1.0,1.0, "LW");
      
      if (kinect->forbiddenZone.size()>=5) 
        cubeVis(viewer, kinect->forbiddenZone[4], 0.0, 1.0,1.0, "RW");
      
   
     while (!viewer->wasStopped ())
      {
        viewer->spinOnce ();
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
   
}
#else
int
main(){
	std::cout << "You should install a video device (X11, GTK, OpenCV, GDI) to run this example" << std::endl;
}
#endif
