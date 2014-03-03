/*!
 * 
 * 
 *  this code read pfm files on the disk using the visp librairie
 *  it calibrate the kinect pose wrt the ground plane in the first image
 *  it assumes that the camera is looking towards the ground
 * 
 *  then it takes all the depthmap files contain in the folder
 *  and using the plane definition of the first image computation
 *  it split the point cloud into plane points in red and green points that are out of the plane
 * 
 */ 




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


void userInput (int argc, char** argv, std::string& path, std::string &pattern, int & nbIm)
{
  
  pattern = "depth_%07d.pfm";
 
  // get the path name
	if (argc>1){
		path = argv[1];
	}
	else {
    path ="/home/dune/Documents/data/kinect/essai1";
  }
  
  // get the number of images to treat
  if (argc>2){
    nbIm = atoi(argv[2]);
    }
  else  nbIm = 50;  
}



boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
float & a, float & b, float & c, float & d, std::string &id)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, id);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,id);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();


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
// -------------
int
main (int argc, char** argv)
{
  // use the shell input
  std::string path, pattern, fullPath;
  int nbIm(0);
  userInput(argc, argv, path, pattern, nbIm);
  
  // create the kinect sensor model to handle the images
  Kinect * kinect= new Kinect();
  kinect->print();
  
  
  // create the pcl viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));   
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

  float minX(-0.2);
  float minY(-0.2);
  float minZ(0.0);
  float maxX(0.2);
  float maxY(0.2);
  float maxZ(1);
  Box box(minX, maxX, minY,maxY,minZ,maxZ);        
          
  std::string cloudId("0");        
  for(int index=0; index<nbIm ; index++){
    // prompt message
    std::cout << "----------------------" << std::endl ;
    std::cout << "Image " << index << std::endl ;
    std::cout << "----------------------" << std::endl ;
        
    // get the point cloud
    Eigen::MatrixXf currentPointCloud = kinect->pointCloud(path,index);
 
    // find the position of the kinect with regards to the ground
    double confidence = 0.02;
    
    // divide the set of points in two parts the point in the plane and
    // the points out of the plane
    Eigen::MatrixXf ptsIn(currentPointCloud), ptsOut(1,3);
    double inlierThres(confidence*2);
  
    std::cout << "Inlier selection" << std::endl;
    box.inlierSelection(ptsIn, ptsOut, inlierThres);
      
    std::cout << "POINT IN "<< ptsIn.rows() << std::endl;
    std::cout << "POINT OUT "<< ptsOut.rows() << std::endl;

    //populate cloud for visualisation    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Conversion::convert(ptsIn,colorCloud, 255, 0, 0);
    Conversion::convert(ptsOut,colorCloud, 0, 255,0);



    // viewer
    // create the id of the current cloud and of the previous one    
    //if (index>=1) viewer->removePointCloud(cloudId);
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    
    
    std::stringstream out;
    out <<  "cloud_" <<index;        
    cloudId = out.str();
         
    std:: cout << " Insertion des points" << std::endl;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colorCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud, rgb,cloudId);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudId);


    viewer->addCube 	(
    box.getMinX(),
    box.getMaxX(),
    box.getMinY(),
    box.getMaxY(),
    box.getMinZ(),
    box.getMaxZ()); 	 
    

      
    int elapse(0);
    //while (!viewer->wasStopped ())
    while (elapse < 10)
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (10000));
      elapse ++;
     std::cout << elapse << std::endl;
    }
  }
}
#else
int
main(){
	std::cout << "You should install a video device (X11, GTK, OpenCV, GDI) to run this example" << std::endl;
}
#endif
