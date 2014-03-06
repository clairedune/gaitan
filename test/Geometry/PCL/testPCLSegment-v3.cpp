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


/*!
 *  align the camera frame with the ground
 */
Eigen::MatrixXf changeFrame(const Eigen::MatrixXf & oP,const Eigen::Matrix4f & wMo)
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




boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
float & a, float & b, float & c, float & d)
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
  
 // coeffs.values.push_back (-0.0478);
 // coeffs.values.push_back (0.3893);
 // coeffs.values.push_back (0.7084);
 // coeffs.values.push_back (-0.5867);
  viewer->addPlane (coeffs, "plane");

  return (viewer);
}

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{

  // segmentation parameters
  // TODO : make a conf file
  float tolerance (0.05); 
  int minClusterSize(100);
  int maxClusterSize(50000);


  //data folder path
  std::string path;
  //data filename
  std::string filename("depth_%07d.pfm"), fullPath;
  //number of images to treat
  int nbIm;
 
  // get the path name
	if (argc>1){
		path = argv[1];
	}
	else {
    path ="/home/dune/Documents/data/kinect/essai1";
  }
  
  // get the number of the image to treat
  if (argc>2){
    nbIm = atoi(argv[2]);
    }
  else  nbIm = 0;
        
      // create the kinect sensor model to handle the images
      double fx(525.0), fy(525.0), cx(319.05), cy(239.5);
      Kinect * kinect= new Kinect(fx,fy,cx,cy);  
        
      // create the point cloud as an eigen matrix
      Eigen::MatrixXf pointCloud = kinect->pointCloud(path,nbIm);
      
      // create a PCL point cloud
      pcl::PointCloud<pcl::PointXYZ>::Ptr simpleCloud(new pcl::PointCloud<pcl::PointXYZ>);
      Conversion::convert(pointCloud,simpleCloud);
      
      //filtering the simple cloud every 1cm
      pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
      kinect->filter( simpleCloud,filteredCloud, 0.01 );
      
      // compute the plane on this filteredcloud
      double confidence(0.02);
      Plane plane = kinect->ground(filteredCloud, confidence);
      plane.print();
      
      // without the filtering does not work
      //Plane plane = kinect->ground(pointCloud, confidence);
      //plane.print();
  
      // select the inliers
      Eigen::MatrixXf ptsIn(pointCloud), ptsOut(3,1) ;
      plane.inlierSelection(ptsIn, ptsOut, confidence);
      
      // align the point cloud with the ground
      Eigen::Matrix4f gMk = plane.computeTransformation();
      Eigen::MatrixXf gPtsIn  = changeFrame(ptsIn,gMk);
      Eigen::MatrixXf gPtsOut = changeFrame(ptsOut,gMk);
      plane.setParameters(0,0,1,0);

      
      //3D Display : populate cloud     
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      Conversion::convert(gPtsIn,colorCloud, 255, 0, 255);
      Conversion::convert(gPtsOut,colorCloud, 0, 255,255);
     
      // keep only the point that are not on ground
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeet(new pcl::PointCloud<pcl::PointXYZ>);
      Conversion::convert(gPtsOut,cloudFeet);
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeetFiltered(new pcl::PointCloud<pcl::PointXYZ>);
      std::vector<pcl::PointIndices> clusterIndices;

      // segment the point that are not on the ground
      kinect->segment(cloudFeet, cloudFeetFiltered,clusterIndices,minClusterSize, maxClusterSize, tolerance);

      // Creating the Clusters
      int j(0);
      for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
         cloudCluster->points.push_back (cloudFeetFiltered->points[*pit]); 
         cloudCluster->width = cloudCluster->points.size ();
         cloudCluster->height = 1;
         cloudCluster->is_dense = true;
       
         if (j==0)
            Conversion::convert(cloudCluster,colorCloud, 255, 0,0);
         else  if (j==0)
            Conversion::convert(cloudCluster,colorCloud, 0,255,0);
         
         std::cout << "PointCloud representing the Cluster: " << cloudCluster->points.size () << " data points." << std::endl;
         j++;
      }
       std::cout << "There are " << j << " clusters " << endl;
      
      
      
      
      
      
      // viewer
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      Eigen::VectorXf param = plane.getParameters(); 
      viewer = shapesVis(colorCloud, param(0), param(1), param(2), param(3));
 
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
