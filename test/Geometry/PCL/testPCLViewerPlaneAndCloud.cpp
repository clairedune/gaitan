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


void convert(const Eigen::MatrixXf & matrix, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const int & r, const int & g, const int & b)
{
 for (int i=0; i<matrix.rows();i++){
            pcl::PointXYZRGB basic_point;
            basic_point.x = matrix(i,0);
            basic_point.y = matrix(i,1);
            basic_point.z = matrix(i,2);
            basic_point.r = r;
            basic_point.g = g;
            basic_point.b = b;
            cloud->push_back(basic_point);
      }
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

std::string filename;

	if (argc>1){
		filename = argv[1];
	}
	else {
		filename="/home/dune/Documents/data/kinect/essai1/depth_0000000.pfm";
  }
	
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
      
      Plane plane;
      
      // recursive plane fitting and outliers selection
      Eigen::MatrixXf ptsIn(pointCloud), ptsOut(3,1),ptsPlane ;
      double confidence(0.15);
      plane.findParameters(ptsIn, ptsOut, confidence);
      confidence/=2;
      plane.findParameters(ptsIn, ptsOut, confidence);
      confidence/=2;
      plane.findParameters(ptsIn, ptsOut, confidence);
   //    confidence/=2;
    //  plane.findParameters(ptsIn, ptsOut, confidence);
      
      //populate cloud     
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =new pcl::PointCloud<pcl::PointXYZRGB>;
      
      convert(ptsIn,cloud, 255, 0, 0);
      convert(ptsOut,cloud, 0, 255,0);
      cloud->width = (int) cloud->points.size ();
      cloud->height = 1;
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      Eigen::VectorXf param = plane.getParameters(); 
      viewer = shapesVis(cloud, param(0), param(1), param(2), param(3));
 
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
