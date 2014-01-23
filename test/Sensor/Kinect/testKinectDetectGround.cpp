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



#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GDI))
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpTime.h>
#include <visp/vpImageIo.h>
#include <visp/vpRGBa.h>

int createRGBaFromFloat(const vpImage<float>&dmap, vpImage<vpRGBa>&Idmap)
{
        int height = dmap.getHeight();
        int width  = dmap.getWidth();
        for(int i = 0 ; i< height ; i++){
          for(int j=0 ; j< width ; j++){
            if (fabs(dmap[i][j] + 1.f) > std::numeric_limits<float>::epsilon()){
              Idmap[i][j].R = (255*dmap[i][j]);
              Idmap[i][j].G = (255*dmap[i][j]);
              Idmap[i][j].B = (255*dmap[i][j]);
            }
            else{
              Idmap[i][j].R = 255;
              Idmap[i][j].G = 0;
              Idmap[i][j].B = 0;
            }
          }
        }
        return 1;
}

int createEigenMatrixFromFloat(const vpImage<float>&dmap, Eigen::MatrixXf & depthMat)
{
      int height = dmap.getHeight();
      int width  = dmap.getWidth();
      for(int i = 0 ; i< height ; i++){
       for(int j=0 ; j< width ; j++){
              depthMat(i,j) = dmap[i][j];
        }
      }
      
      return 1;
}

int main(int argc, char ** argv) {
	
	std::string filename;

	if (argc>1){
		filename = argv[1];
	}
	else {
		filename="/home/dune/Documents/data/kinect/essai1/depth_0000001.pfm";
	}
	
	try {
      vpImage<float> dmap(480,640);//for medium resolution
      vpImage<vpRGBa> Idmap(480,640);
      
      vpDisplayX display;
      display.init(Idmap, 100, 200,"Depth map");
      
    
      // read the image
      try{
          vpImageIo::readPFM(dmap,filename.c_str());
        }
      catch(...){
              std::cout << "Catch an exception when reading image " << filename << std::endl;
      }
       
      // copy the matrix in an eigen mat
      Eigen::MatrixXf depthMat;
      createEigenMatrixFromFloat(dmap, depthMat);

      // kinect internal parameters
      float fx(525.0), fy(525.0), cx(319.05), cy(239.5);
      Eigen::Vector4f *point3D;
      int index(0);
      for (int i=0; i<depthMat.rows();i++)
        for (int j=0 ; j<depthMat.cols();j++)
        {
          point3D[index](2) = depthMat(i,j)/1000; //to convert into meters 
          point3D[index](0) = (i-cx)*point3D[index](2)/fx; 
          point3D[index](1) = (j-cy)*point3D[index](2)/fy;
          point3D[index](3) = 1; 
          index++;
        }
      
      
  
     // create Idmap for displaying
     createRGBaFromFloat(dmap, Idmap);

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
