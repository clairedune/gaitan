#include <visp/vpConfig.h>

#include <sys/time.h> // to get time
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <math.h>


#ifdef VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES
#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GDI))
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpKinect.h>
#include <visp/vpTime.h>
#include <visp/vpImageIo.h>


int main(int argc, char ** argv) {
	
	std::string path, 
				rgbFilename("RGB_%07d.ppm"), 
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
      std::string timeFullPath = path + "/"+ timeFilename; 
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
              file.flush();
			      }
 
        
        vpDisplay::display(Idmap);
        vpDisplay::flush(Idmap);
        vpDisplay::display(Irgb);
        vpDisplay::flush(Irgb);
        
        // save the depth image
        std::string depthPath = path+"/"+depthFilename;
        char buf[100];
        sprintf(buf,depthPath.c_str(),index);
        std::string depthFullPath(buf);
        std::cout << "Write in : " << depthFullPath << std::endl;
        try{
          vpImageIo::writePFM(dmap,depthFullPath.c_str());
        }
        catch(...){
              std::cout << "Catch an exception when writing image " << depthFullPath << std::endl;
        }
        //save the color image
        std::string rgbPath = path+"/"+rgbFilename;
        sprintf(buf,rgbPath.c_str(),index);
        std::string rgbFullPath(buf);
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
    std::cout << "The time file is closed" << std::endl;
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
#else
int
main(){
	std::cout << "You should install libfreenect to run this example" << std::endl;
}
#endif
