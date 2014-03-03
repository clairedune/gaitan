/* \author Geoffrey Biggs */


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
  
  std::string filename;

	if (argc>1){
		filename = argv[1];
	}
	else {
		filename="/home/dune/Documents/data/kinect/essai1/depth_0000000.pfm";
  }
  
  std::cout << "filename : "<< filename << std::endl;   
	
	try {
    
      int width(640), height(480);
      vpImage<float> dmap(height,width);//for medium resolution

      // read the image
      try{
          vpImageIo::readPFM(dmap,filename.c_str());
        }
      catch(...){
              std::cout << "Catch an exception when reading image " << filename << std::endl;
      }
        
      // copy the matrix in an eigen mat
      Eigen::MatrixXf depthMap, pointCloud;
      Conversion::convert(dmap, depthMap);
      double fx(525.0), fy(525.0), cx(319.05), cy(239.5);
      Conversion::convert(depthMap,pointCloud,fx,fy,cx,cy);

      std::cout << "Point cloud size : " << pointCloud.rows()<< std::endl;

      // find the coeff of the main plane
      Eigen::MatrixXf ptsIn(pointCloud), ptsOut(3,1) ;
      double confidence(0.02);
      pcl::PointCloud<pcl::PointXYZ>::Ptr simpleCloud(new pcl::PointCloud<pcl::PointXYZ>);
      Conversion::convert(pointCloud,simpleCloud);
      Plane plane;
      plane.findParameters(simpleCloud,confidence);
      plane.print();
      
      // divide the point cloud into two clouds.
      plane.inlierSelection(ptsIn, ptsOut, confidence);
    
      //change point frame
      Eigen::Matrix4f gMk = plane.computeTransformation();
      plane.changeFrame(gMk);
      Eigen::MatrixXf gPtsIn  = changeFrame(ptsIn,gMk);
      Eigen::MatrixXf gPtsOut = changeFrame(ptsOut,gMk);
      
      //populate cloud     
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
     // Conversion::convert(ptsIn,colorCloud, 255, 0, 0);
      //Conversion::convert(ptsOut,colorCloud, 0, 255,0);
      Conversion::convert(gPtsIn,colorCloud, 255, 0, 255);
      Conversion::convert(gPtsOut,colorCloud, 0, 255,255);
      //colorCloud->width = (int) colorCloud->points.size();
      //colorCloud->height = 1;
      
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      Conversion::convert(gPtsOut,cloud);
      std::cout << "PointCloud before filtering has: " << cloud->points.size ()  << " data points." << std::endl; //*
      
      // Create the filtering object: downsample the dataset using a leaf size of 1cm
      pcl::VoxelGrid<pcl::PointXYZ> vg;
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);
      vg.setInputCloud (cloud);
      vg.setLeafSize (0.01f, 0.01f, 0.01f);
      vg.filter (*cloudFiltered);
      std::cout << "PointCloud after filtering has: " << cloudFiltered->points.size ()  << " data points." << std::endl; //*

      
      // Creating the KdTree object for the search method of the extraction
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      tree->setInputCloud (cloudFiltered);     
      std::vector<pcl::PointIndices> clusterIndices;
      pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
      ec.setClusterTolerance (tolerance); // 5cm
      ec.setMinClusterSize (minClusterSize);
      ec.setMaxClusterSize (maxClusterSize);
      ec.setSearchMethod (tree);
      ec.setInputCloud (cloudFiltered);
      ec.extract (clusterIndices);


      // Creating the Clusters
      int j(0);
      for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
      {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
         cloudCluster->points.push_back (cloudFiltered->points[*pit]); 
         cloudCluster->width = cloudCluster->points.size ();
         cloudCluster->height = 1;
         cloudCluster->is_dense = true;
       
         if(j<2)
         Conversion::convert(cloudCluster,colorCloud, j*100, j*100,255);
       
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
	  catch(vpException e) {
		  std::cout << "Catch an exception: " << e << std::endl;
		  return -1;
	  }
	  catch(...){
		  std::cout << "Catch an exception " << std::endl;
		  return -1;
	  }
}
#else
int
main(){
	std::cout << "You should install a video device (X11, GTK, OpenCV, GDI) to run this example" << std::endl;
}
#endif
