/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// TODO : everything in pcl?


#include <libgaitan/kinect.h>
#include <libgaitan/conversion.h>
#include <libgaitan/plane.h>

#include <visp/vpConfig.h>
#include <sys/time.h> // to get time
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <math.h>

#include <Eigen/Dense>


#ifdef VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES
#include <visp/vpImage.h>
#include <visp/vpImageIo.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpKinect.h>
#include <visp/vpTime.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

namespace gaitan
{
   
   /*!
   * \brief kinect constructor with defaut kinect parameters
   */
   Kinect::Kinect() : RGBDSensor(594.0,591.0,339.05, 247.5)
   {
     return;
   }

      
   Kinect::Kinect(double fx, double fy, double cx, double cy) 
  {  
   this->fx = fx ; 
   this->fy = fy ; 
   this->cx = cx ;  
   this->cy = cy ;
  }
 
 
  Kinect::~Kinect()
  {
  
  }
  
  int  Kinect::display(){
    try {
    	Freenect::Freenect freenect;
    	vpKinect & kinect = freenect.createDevice<vpKinect>(0);
    	// Start acquisition thread with a depth map resolution of 480x640
    	kinect.start(vpKinect::DMAP_MEDIUM_RES); 
    
    	vpImage<unsigned char> Idmap(480,640);//for medium resolution
    	vpImage<float> dmap(480,640);//for medium resolution
    	vpImage<vpRGBa> Irgb(480,640);
		  vpDisplayX display, displayRgb;
      display.init(Idmap, 100, 200,"Depth map");
		  displayRgb.init(Irgb, 900, 200,"Color Image");

		  // A click to stop acquisition
		  std::cout << "Click in one image to stop acquisition" << std::endl;
		  while(!vpDisplay::getClick(Idmap,false) && !vpDisplay::getClick(Irgb,false)){
  			kinect.getDepthMap(dmap); 
  			kinect.getDepthMap(dmap, Idmap);
  			kinect.getRGB(Irgb);
  			vpDisplay::display(Idmap);
  			vpDisplay::flush(Idmap);
  			vpDisplay::display(Irgb);
  			vpDisplay::flush(Irgb);
  		  }
  		  std::cout << "Stop acquisition" << std::endl;
  		  kinect.stop(); // Stop acquisition thread
  		  return 0;
      }	
	    catch(vpException e) {
      std::cout << "Catch an exception: " << e << std::endl;
		  return 1;
	    }
    
    }
    
    
  /*!
   * 
   * Use Visp Class to acquire the data and store them in pfm files
   * 
   */   
  
  int Kinect::acquire(const std::string &path , bool flagDisp)
  {

	    try {
    	  Freenect::Freenect freenect;
    
    	  vpKinect & kinect = freenect.createDevice<vpKinect>(0);
    	  // Start acquisition thread with a depth map resolution of 480x640
    	  kinect.start(vpKinect::DMAP_MEDIUM_RES); 
    
    	  vpImage<unsigned char> Idmap(480,640);//for medium resolution
    	  vpImage<float> dmap(480,640);//for medium resolution
    	  vpImage<vpRGBa> Irgb(480,640);
    	
		    vpDisplayX display, displayRgb;
 		    display.init(Idmap, 100, 200,"Depth map");
		    displayRgb.init(Irgb, 900, 200,"Color Image");
      
        //init val
        int index(0);
        struct timeval now ;
      
        //open time file
        std::string timeFullPath = this->timePath(path);
        std::ofstream file(timeFullPath.c_str(), std::ios::out);
        int precision(15);
       
        // A click to stop acquisition
        std::cout << "Click in one image to stop acquisition" << std::endl;
        while(!vpDisplay::getClick(Idmap,false) && !vpDisplay::getClick(Irgb,false)){
          kinect.getDepthMap(dmap);
          kinect.getDepthMap(dmap, Idmap);
          kinect.getRGB(Irgb);
        
          // get the current time
          gettimeofday(&now,NULL);
          double kinectTime = now.tv_sec + ((double)now.tv_usec)/1000000.0;
        
          if(file){
	  			    file << index << "\t";
              file << std::setprecision(precision) <<kinectTime << "\n";	 
			      }
 
          if(flagDisp){
            vpDisplay::display(Idmap);
            vpDisplay::flush(Idmap);
            vpDisplay::display(Irgb);
            vpDisplay::flush(Irgb);
          }
          else
          {
            std::string texte("Display OFF");
            vpColor c(255,0,0);
            vpDisplay::displayCharString(Idmap, 50,50,texte.c_str(),c);
            vpDisplay::displayCharString(Irgb, 50,50,texte.c_str(),c);
            vpDisplay::flush(Idmap);
            vpDisplay::flush(Irgb);
          }
        
          // save the depth image
          std::string depthFullPath = this->depthPath(path, index);
          if(flagDisp)
            std::cout << "Write in : " << depthFullPath << std::endl;
          try{
            vpImageIo::writePFM(dmap,depthFullPath.c_str());
          }
          catch(...){
              std::cout << "Catch an exception when writing image " << depthFullPath << std::endl;
          }
          //save the color image
          std::string rgbFullPath = this->rgbPath(path, index);
          if(flagDisp)
            std::cout << "Write in : " << rgbFullPath << std::endl;
          try{
            vpImageIo::writePPM(Irgb,rgbFullPath.c_str());
          }
          catch(...){
              std::cout << "Catch an exception when writing image " << rgbFullPath << std::endl;
          }
        
          index++;
		    }
    
		    std::cout << "Stop acquisition" << std::endl;
		    kinect.stop(); // Stop acquisition thread
        if  (file) file.close();
		    return 1;
	    }	
	    catch(vpException e) {
		    std::cout << "Catch an exception: " << e << std::endl;
        return 0;
      }

  }
  
  
Eigen::MatrixXf Kinect::pointCloud(const std::string & path,const int &index)
{
      Eigen::MatrixXf pts3D;
      // create the visp image to read the visp file          
      // FIXME may be not re alluy good to fix the size here ...
      int width(640), height(480);
      vpImage<float> dmap(height,width);//for medium resolion 
      
      // build the filename
      std::string filename =  this->depthPath(path, index);
      std::cout << "filename  : " << filename << std::endl;
     
      try{
            vpImageIo::readPFM(dmap,filename.c_str());
      }
      catch(...){
               std::cerr << "ERROR KINECT::DEPTHMAP >> Catch an exception when reading image " << filename << std::endl;
        }

     Conversion::convert( dmap , pts3D , this->fx, this->fy, this->cx, this->cy);
    
      return pts3D;
}
    
}
#endif //libfreenect
