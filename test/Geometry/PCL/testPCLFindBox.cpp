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
  //pcl::ModelCoefficients coeffs;
  //coeffs.values.push_back (a);
  //coeffs.values.push_back (b);
  //coeffs.values.push_back (c);
  //coeffs.values.push_back (d);
 //viewer->addPlane (coeffs, "plane");

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
  int nbIm;
  userInput(argc, argv, path, pattern, nbIm);
     
   
  // ------------ SET UP ------------------------------------//
  
  // segmentation parameters
  // TODO : make a conf file
  float clusterTolerance (0.04); 
  int minClusterSize(150);
  int maxClusterSize(50000);
  double confidence(0.02);
  float leafSize(0.008); 
   
  //-------------------------------------------------------------//
        
  // create the kinect sensor model to handle the images
  Kinect * kinect= new Kinect();  
      
  // create the point cloud as an eigen matrix
  Eigen::MatrixXf pointCloud = kinect->pointCloud(path,nbIm);
      
  // detect clusters
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeetFiltered (new pcl::PointCloud<pcl::PointXYZ>);
  std::vector<pcl::PointIndices> clusterIndices;
  kinect->detectClusters(pointCloud, cloudFeetFiltered, clusterIndices,clusterTolerance, 
                         minClusterSize,
                         maxClusterSize,
                         confidence, 
                         leafSize);    
    
    
  int j=0;
  
  Box leftFoot, rightFoot, leftWheel, rightWheel;
  
  // Creating the Clusters
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
         cloudCluster->points.push_back (cloudFeetFiltered->points[*pit]); 
         cloudCluster->width = cloudCluster->points.size ();
         cloudCluster->height = 1;
         cloudCluster->is_dense = true;
         
         
         // create a MAtrix clusterot
         Eigen::MatrixXf ptsCluster;
         Conversion::convert(cloudCluster, ptsCluster);                    
         
         if (j==0){
            //Conversion::convert(cloudCluster,colorCloud, 255, 0,0);
            leftFoot.findParameters(ptsCluster);
            leftFoot.print();
          }
            
            
         else  if (j==1){
            //Conversion::convert(cloudCluster,colorCloud, 0,255,0);
            rightFoot.findParameters(ptsCluster);
          }
          
         else  if (j==2){
            //Conversion::convert(cloudCluster,colorCloud, 0,255,255);
            leftWheel.findParameters(ptsCluster);
          }
         else  if (j==3){
            //Conversion::convert(cloudCluster,colorCloud, 0,255,255);
            rightWheel.findParameters(ptsCluster);
          }
          
          
         else 
            Conversion::convert(cloudCluster,colorCloud, 0,0,50*j);
         
         std::cout << "PointCloud representing the Cluster: " << cloudCluster->points.size () << " data points." << std::endl;
         j++;
      }
    std::cout << "There are " << j << " clusters " << endl;
      
      
   // Eigen::MatrixXf ptsIn(pointCloud), ptsOut(1,3);
   // double inlierThres(confidence*2);
  
//    leftWheel.inlierSelection(ptsIn, ptsOut, inlierThres);
 //   Conversion::convert(ptsIn,colorCloud, 255,0,0);
    
   // ptsIn = ptsOut;
    
    //rightWheel.inlierSelection(ptsIn, ptsOut, inlierThres);
    //Conversion::convert(ptsIn, colorCloud, 255,0,0);
    //Conversion::convert(ptsOut, colorCloud, 0,255,0);
    
      
      
      
      // viewer
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      viewer = shapesVis(colorCloud,0.0f, 0.0f, 1.0f, 0.0f);
      cubeVis(viewer, leftFoot, 1.0, 0.0,0.0, "LF");
      cubeVis(viewer, rightFoot, 0.0, 1.0,0.0, "RF");
      cubeVis(viewer, leftWheel, 0.0, 1.0,1.0, "LW");
      cubeVis(viewer, rightWheel, 0.0, 1.0,1.0, "RW");
      
	    
   
      while (!viewer->wasStopped ())
      {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (100000));
      }
   
}
#else
int
main(){
	std::cout << "You should install a video device (X11, GTK, OpenCV, GDI) to run this example" << std::endl;
}
#endif
