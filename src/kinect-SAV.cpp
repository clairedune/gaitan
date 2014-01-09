#include <iostream>
#include <string>



#include <libgaitan/kinect.h>


namespace gaitan
{
  Kinect::Kinect()
  {
    std:: cout << "construction" <<endl; 
  }

 }




/*
#include <visp/vpConfig.h>

// if Visp has found libfreenect
#ifdef VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES
#include <visp/vpKinect.h>
#include <visp/vpTime.h>
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>


  int Kinect::acquire(string path)
  {
    // create the kinect grabber
    Freenect::Freenect freenect;
    vpKinect & kinect = freenect.createDevice<vpKinect>(0);
    kinect.start(vpKinect::DMAP_MEDIUM_RES); // Start acquisition thread with a depth map resolution of 480x640
    vpImage<unsigned char> Idmap(480,640);//for medium resolution
    vpImage<float> dmap(480,640);//for medium resolution
    vpImage<vpRGBa> Irgb(480,640);

    vpDisplayX display, displayRgb;
    display.init(Idmap, 100, 200,"Depth map");
    displayRgb.init(Irgb, 900, 200,"Color Image");


    // A click to stop acquisition
    std::cout << "Click in one image to stop acquisition" << std::endl;
    while(!vpDisplay::getClick(Idmap,false) && !vpDisplay::getClick(Irgb,false))
    {
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

}

#else
namespace gaitan
{
  Kinect::Kinect()
  {
    cerr << "libfreenect not found" << endl;
  }


  int Kinect::acquire(string path)
  {
    returnr0;
  }
}

#endif // if visp has found kinect*/
