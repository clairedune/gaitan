#include <visp/vpConfig.h>
#include <iostream>
#ifdef VISP_HAVE_LIBFREENECT_AND_DEPENDENCIES
#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GDI))
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpKinect.h>
#include <visp/vpTime.h>
int main() {
try {
// Init Kinect
#ifdef VISP_HAVE_LIBFREENECT_OLD
// This is the way to initialize Freenect with an old version of libfreenect packages under ubuntu lucid 10.04
Freenect::Freenect<vpKinect> freenect;
vpKinect & kinect = freenect.createDevice(0);
#else
Freenect::Freenect freenect;
vpKinect & kinect = freenect.createDevice<vpKinect>(0);
#endif
// Set tilt angle in degrees
if (0) {
float angle = -3;
kinect.setTiltDegrees(angle);
}
// Init display
#if 1
kinect.start(vpKinect::DMAP_MEDIUM_RES); // Start acquisition thread with a depth map resolution of 480x640
vpImage<unsigned char> Idmap(480,640);//for medium resolution
vpImage<float> dmap(480,640);//for medium resolution
#else
kinect.start(vpKinect::DMAP_LOW_RES); // Start acquisition thread with a depth map resolution of 240x320 (default resolution)
vpImage<unsigned char> Idmap(240,320);//for low resolution
vpImage<float> dmap(240,320);//for low resolution
#endif
vpImage<vpRGBa> Irgb(480,640),Iwarped(480,640);
#if defined VISP_HAVE_X11
vpDisplayX display, displayRgb, displayRgbWarped;
#elif defined VISP_HAVE_GTK
vpDisplayGTK display;
vpDisplayGTK displayRgb;
vpDisplayGTK displayRgbWarped;
#elif defined VISP_HAVE_OPENCV
vpDisplayOpenCV display;
vpDisplayOpenCV displayRgb;
vpDisplayOpenCV displayRgbWarped;
#elif defined VISP_HAVE_GDI
vpDisplayGDI display;
vpDisplayGDI displayRgb;
vpDisplayGDI displayRgbWarped;
#endif
display.init(Idmap, 100, 200,"Depth map");
displayRgb.init(Irgb, 900, 200,"Color Image");
displayRgbWarped.init(Iwarped,900,700,"Warped Color Image");
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
//Warped RGB image:
kinect.warpRGBFrame(Irgb,dmap, Iwarped);
vpDisplay::display(Iwarped);
vpDisplay::flush(Iwarped);
}
std::cout << "Stop acquisition" << std::endl;
kinect.stop(); // Stop acquisition thread
return 0;
}
catch(vpException e) {
std::cout << "Catch an exception: " << e << std::endl;
return 1;
}
catch(...) {
std::cout << "Catch an exception " << std::endl;
return 1;
}
}
#else
int
main()
{
std::cout << "You should install a video device (X11, GTK, OpenCV, GDI) to run this example" << std::endl;
}
#endif
#else
int
main()
{
std::cout << "You should install libfreenect to run this example" << std::endl;
}
#endif
