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
 *  v2 : this time, we use directly the position calibration of the kinect to select the inliers 
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


void userInput (int argc, char** argv, std::string& path, std::string &pattern, int & beg, int&end)
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
    beg = atoi(argv[2]);
    }
  else  {
    beg = 0;
    end =10;
  }
  if (argc>3){
    end = atoi(argv[3]);
    }
  else  {
    end =10;
  }
  
    
}


// --------------
// -----Main-----
// -------------
int
main (int argc, char** argv)
{
  // use the shell input
  std::string path, pattern, fullPath;
  int begIm(0), endIm(0);
  userInput(argc, argv, path, pattern,begIm,endIm );
  
  
  // create the kinect sensor model to handle the images
  Kinect * kinect= new Kinect();
  
  
  // create the pcl viewer
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));   
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  
  
  
  // ground plane : assumed to be the same for all images
  Plane gPlane(0,0,1,0);      
          
          
  std::string cloudId("0");  
  int firstIm(true);
       
  for(int index=begIm; index<endIm ; index++){

    // prompt message
    std::cout << "----------------------" << std::endl ;
    std::cout << "Image " << index << std::endl ;
    std::cout << "----------------------" << std::endl ;
            
    // get the point cloud
    Eigen::MatrixXf currentPointCloud = kinect->pointCloud(path,index);
 
    // find the position of the kinect with regards to the ground
    double confidence = 0.02;
    if(firstIm){
      //plane = kinect->ground(currentPointCloud, confidence);
      kinect->extrinsicCalibration(currentPointCloud,confidence);
      firstIm=false;
    }
      kinect->print();
 
    
    
    Eigen::MatrixXf  gCurrentPointCloud = kinect->changeFrame(currentPointCloud);  

    // divide the set of points in two parts the point in the plane and
    // the points out of the plane
    Eigen::MatrixXf gPin(gCurrentPointCloud), gPout(3,1);
    double inlierThres(confidence*2);
    gPlane.inlierSelection(gPin, gPout, inlierThres);
    
    std::cout << "POINT IN "<< gPin.rows() << std::endl;
    std::cout << "POINT OUT "<< gPout.rows() << std::endl;

    //populate cloud for visualisation    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Conversion::convert(gPin,colorCloud, 255, 0, 0);
    Conversion::convert(gPout,colorCloud, 0, 255,0);
    
    // viewer
    // create the id of the current cloud and of the previous one    
    if (index>=1) viewer->removePointCloud(cloudId);
    
    std::stringstream out;
    out <<  "cloud_" <<index;        
    cloudId = out.str();
         
    std:: cout << " Insertion des points" << std::endl;
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colorCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud, rgb,cloudId);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudId);

      
    int elapse(0);
    //while (!viewer->wasStopped ())
    while (elapse < 3)
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
