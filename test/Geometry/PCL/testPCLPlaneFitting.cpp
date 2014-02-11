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
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <time.h>


boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
double a, double b, double c, double  d)
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
    
    
      //populate cloud     
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
      Conversion::convert(pointCloud, cloud);
      cloud->width = (int) cloud->points.size ();
      cloud->height = 1;
      
      
      

      Plane plane;
      
      time_t start,end;
      double dif;
   
   
      double distanceThreshold(0.001);      
      //evaluate time
      time (&start);
      plane.findParameters(cloud, distanceThreshold);
      //evaluate time
      time (&end);
      dif = difftime (end,start);
      std::cout <<  "  La methode ransac de la pcl  " << std::endl;
      printf ("Elasped time is %.2lf seconds.", dif );
      plane.print();
      
   
      double confidence(0.15);
      Eigen::MatrixXf ptsIn(pointCloud), ptsOut(3,1) ;
      //evaluate time
      time (&start);
      // launch method
      plane.findParameters(ptsIn, ptsOut, confidence);
      // evaluate time
      time (&end);
      dif = difftime (end,start);
      printf ("Elasped time is %.2lf seconds.", dif );
      std::cout <<  "  La methode de la classe plane  " << std::endl; 
      plane.print(); 

       
       
      
       
      
      distanceThreshold=0.02;
      plane.inlierSelection(ptsIn, ptsOut, distanceThreshold);

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      Conversion::convert(ptsIn, colorCloud, 255, 0,0);
      Conversion::convert(ptsOut, colorCloud, 0,255,0);
   
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
      viewer = shapesVis (colorCloud,plane.getA(),plane.getB(),plane.getC(),plane.getD());
       
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
