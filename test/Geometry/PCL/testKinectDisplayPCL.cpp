/*!
 * Read the kinect data saved in a directory 
 * given as an input to the test
 * 
 * 
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





#include <pcl/visualization/cloud_viewer.h>

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
 

int main(int argc, char ** argv) {
	
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
      vpImage<vpRGBa> Idmap(height,width);
      
      vpDisplayX display;
      display.init(Idmap, 100, 200,"Depth map");
      
      // convert to table and save.
   
      // read the image
      try{
          vpImageIo::readPFM(dmap,filename.c_str());
        }
      catch(...){
              std::cout << "Catch an exception when reading image " << filename << std::endl;
      }
      
      // create Idmap for displaying
      Conversion::convert(dmap, Idmap);
  
      // copy the matrix in an eigen mat
      Eigen::MatrixXf depthMap, pointCloud;
      Conversion::convert(dmap, depthMap);
      double fx(525.0), fy(525.0), cx(319.05), cy(239.5);
      
      Conversion::convert(depthMap,pointCloud,fx,fy,cx,cy);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
      
      Plane plane;//(0.0842, -0.6701,-1.2099,-1.00); 
      // recursive plane fitting and outliers selection
      Eigen::MatrixXf ptsIn(pointCloud), ptsOut(3,1),ptsPlane ;
      double confidence(0.01);
      plane.findParameters(ptsIn, ptsOut, confidence);
      plane.print(); 
      
      //Conversion::convert(pointCloud, depthMap,height,width, fx, fy, cx, cy);
      //Conversion::convert(depthMap, dmap);
      
       //populate cloud
      plane.createPointCloud(ptsPlane);
      convert(ptsPlane, cloud,255,255,255);     
      convert(ptsIn,cloud, 255, 0, 0);
      convert(ptsOut,cloud, 0, 255,0);
     // pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
     // viewer.showCloud(cloud);

     
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
    //cloud->points[cloud->size() - 1], "line");
    //viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

    //---------------------------------------
    //-----Add shapes at other locations-----
    //---------------------------------------
    pcl::ModelCoefficients coeffs;
    Eigen::VectorXf param = plane.getParameters();
    coeffs.values.push_back (param(0));
    coeffs.values.push_back (param(1));
    coeffs.values.push_back (param(2));
    coeffs.values.push_back (param(3));
    viewer->addPlane (coeffs, "plane");
     
    while (!viewer->wasStopped ()){
      } 
     
          
      
      // create Idmap for displaying
      Conversion::convert(dmap, Idmap);
      
      //Draw in an out points
       for (int index=0; index<ptsIn.rows();index++)
        {
          float val = ptsIn(index,2)*1000;
          int   i   = round(fx*ptsIn(index,0)/ptsIn(index,2)+cx);
          int   j   = round(fy*ptsIn(index,1)/ptsIn(index,2)+cy);
          if(i>=0 && i<height && j>=0 && j<width){
            Idmap[i][j].R = 255;
            Idmap[i][j].G = 0;
            Idmap[i][j].B = 0;
          }
          index++;
        } 
       for (int index=0; index<ptsOut.rows();index++)
        {
          float val = ptsOut(index,2)*1000;
          int   i   = round(fx*ptsOut(index,0)/ptsOut(index,2)+cx);
          int   j   = round(fy*ptsOut(index,1)/ptsOut(index,2)+cy);
           if(i>=0 && i<height && j>=0 && j<width){
          Idmap[i][j].R = 0;
          Idmap[i][j].G = 255;
          Idmap[i][j].B = 0;
          }
          index++;
        } 


     // display image
     vpDisplay::display(Idmap);
     vpDisplay::flush(Idmap);
     
        
      // click to go to next image
      while(!vpDisplay::getClick(Idmap,false)){
      }
	
  
  	std::cout << "Finish" << std::endl;
		return 0;
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

