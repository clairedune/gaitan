#include <visp/vpConfig.h>
#include <iostream>
#include <fstream>

#ifdef VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES
#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GDI))
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpKinect.h>
#include <visp/vpTime.h>



int main(int argc, char ** argv) {
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
