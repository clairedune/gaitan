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


int main(int argc, char ** argv) {
	
	std::string path, 
				depthFilename("depth_%07d.pfm"),
				timeFilename("TimeSampling.dat");

	if (argc>1){
		path = argv[1];
	}
	else {
		std::cerr << "ERROR Usage : " << argv[0] << " pathname" << std::endl;
		return -1;
	}
	
	try {
      vpImage<float> dmap(480,640);//for medium resolution
      vpImage<vpRGBa> Idmap(480,640);
      
      vpDisplayX display;
      display.init(Idmap, 100, 200,"Depth map");
      
      //init val
      int index(0);
      float kinectTime(0.0);
      
      //open time file
      std::string timeFullPath = path + "/"+ timeFilename; 
      std::ifstream file(timeFullPath.c_str(), std::ios::in);
       
      //count lines
      int nbImages(0);
      std::string line;
      if (file){
        while(getline(file, line)) nbImages++;      
        file.clear();
        file.seekg(0,std::ios::beg); 
      }       
       
      // A click to view next image
      std::cout << "Click in one image to open next image" << std::endl;
      while(index<nbImages){
        
        if(file){
	  			    file >> index ;
              file >> kinectTime;	 
        }
        
        std::cout << "Index : "<< index << " \t Time : " << std::setprecision(15)<<kinectTime << std::endl;
            
        // compose file name    
        std::string depthPath = path+"/"+depthFilename;
        char buf[100];
        sprintf(buf,depthPath.c_str(),index);
        std::string depthFullPath(buf);
        
        // read the image
        try{
          vpImageIo::readPFM(dmap,depthFullPath.c_str());
        }
        catch(...){
              std::cout << "Catch an exception when reading image " << depthFullPath << std::endl;
        }
        
        // create Idmap for displaying
        createRGBaFromFloat(dmap, Idmap);

        // display image
        vpDisplay::display(Idmap);
        vpDisplay::flush(Idmap);
        
        // click to go to next image
        while(!vpDisplay::getClick(Idmap,false)){
        }
		}
    
		std::cout << "Stop reading" << std::endl;
    if  (file) file.close();
		return 0;
	}	
	catch(vpException e) {
		std::cout << "Catch an exception: " << e << std::endl;
		return 1;
	}
	catch(...){
		std::cout << "Catch an exception " << std::endl;
		return 1;
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
