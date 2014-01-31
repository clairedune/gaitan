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




int main(int argc, char ** argv) {
	
	std::string filename;

	if (argc>1){
		filename = argv[1];
	}
	else {
		filename="/home/dune/Documents/data/kinect/essai1/depth_0000001.pfm";
	}
	
	try {
    
      int width(640), height(480);
      vpImage<float> dmap(height,width);//for medium resolution
      vpImage<vpRGBa> Idmap(height,width);
      
      vpDisplayX display;
      display.init(Idmap, 100, 200,"Depth map");
      
      
   
      // read the image
      try{
          vpImageIo::readPFM(dmap,filename.c_str());
        }
      catch(...){
              std::cout << "Catch an exception when reading image " << filename << std::endl;
      }
      
      // create Idmap for displaying
      Conversion::convert(dmap, Idmap);
      vpDisplay::display(Idmap);
      vpDisplay::flush(Idmap);
    
         
     // click to go to next image
      while(!vpDisplay::getClick(Idmap,false)){
        //wait
      }
    
    
      // copy the matrix in an eigen mat
      Eigen::MatrixXf depthMap, pointCloud;
      Conversion::convert(dmap, depthMap);
      double fx(525.0), fy(525.0), cx(319.05), cy(239.5);
      Conversion::convert(depthMap, pointCloud,fx,fy,cx,cy);
      
      Plane plane; 
      // recursive plane fitting and outliers selection
      Eigen::MatrixXf ptsIn(pointCloud), ptsOut(3,1);
      double confidence(0.005);
    
      plane.findParameters(ptsIn); 
      plane.inlierSelection(ptsIn,ptsOut, confidence);
      //plane.findParameters(ptsIn, ptsOut, confidence);
  
      plane.print(); 
      
      //Conversion::convert(pointCloud, depthMap,height,width, fx, fy, cx, cy);
      //Conversion::convert(depthMap, dmap);
      
      // create Idmap for displaying
      Conversion::convert(dmap, Idmap);
      
      //Draw in an out points
       for (int index=0; index<ptsIn.rows();index++)
        {
          float val = ptsIn(index,2)*1000;
          int   i   = round(fx*ptsIn(index,0)/ptsIn(index,2)+cx);
          int   j   = round(fy*ptsIn(index,1)/ptsIn(index,2)+cy);
          if(i>=0 && i<height && j>=0 && j<width){
            Idmap[i][j].R = 255*val;
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
          Idmap[i][j].G = 255*val;
          Idmap[i][j].B = 0;
          }
          index++;
        } 
     
     
     // click to go to next image
      while(!vpDisplay::getClick(Idmap,false)){
        //wait
      }

     // display image
     vpDisplay::display(Idmap);
     vpDisplay::flush(Idmap);
     
        
      // click to go to next image
      while(!vpDisplay::getClick(Idmap,false)){
        //wait
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

#ifndef GAITAN_KINECT_H
#define GAITAN_KINECT_H

#include <libgaitan/sensor.h>
#include <libgaitan/table.h>



namespace gaitan
{
  class Kinect:public Sensor
  {   
    public:	
     Kinect();
     Kinect(string path);
     ~Kinect();
      
  };
}
#endif // kinect_H
