#include <iostream>
#include <fstream>
#include <string>
#include <OpenNI.h>

using namespace std;
using namespace openni;
  
int main(int argc, char **argv) {
  std::cout << std::endl 
            << "----- START TEST OPEN NI----"
            << std::endl;

  // init the sensors
  OpenNI::initialize();  
  //std::string message = OpenNI::getExtendedError();
  //std::cout << "Connection error : " << message << std:: endl;
  std::cout << "Initialisation ok" << std::endl;
  
  //list of devices
  Array<DeviceInfo> deviceList;
  OpenNI::enumerateDevices(&deviceList);
  // display all the device name
  
  if (deviceList.getSize()==0){
    cout << "no device detected"<< endl; 
    return -1;
  }
  
  
  for(int i=0; i< deviceList.getSize() ; i++)
    std::cout << deviceList[i].getName() << std::endl;
  
  //use of any connected device 
  Device device;
  if (device.open(ANY_DEVICE)!=STATUS_OK){
    cout << "error opening device"<< endl; 
    return -1;
  }
  else
  {
    cout << "device is openned"<< endl;
    }
  
  
  device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR );
  device.setDepthColorSyncEnabled 	( TRUE) ;	
  
  std::cout << "Open device ok" << std::endl;
  
  
  
  // create video stream
  VideoStream depth, color;
  if(depth.create(device, SENSOR_DEPTH)!=STATUS_OK) {
     cout << "erreur a la creation"<< endl;
     return -2;
     }
  else{
     cout << "Ok for depth sensor" << endl;
  }  
  
  if(color.create(device, SENSOR_COLOR)!=STATUS_OK) {
     cout << "erreur a la creation"<< endl;
     return -3;
  }
  else{
    cout << "Ok for color sensor" << endl;
  }  
     
      
  
  
  //start the depth stream and check if it has really started
  if(depth.start() != openni::STATUS_OK){
    std::cout << "The depth stream did not start"<< std::endl;
    return -2;
  }
  else{
      std::cout << "The deph stream did start"<< std::endl;
  }
  std::cout << "Start depth checked" << std::endl;

  VideoFrameRef depthFrame;    //IR VideoFrame Class Object
  VideoMode vmode;      // VideoMode Object
  Status rc = STATUS_OK;
  while(true)            
  {
    if(device.getSensorInfo(SENSOR_DEPTH) != NULL)
    {
        rc = depth.readFrame(&depthFrame);        // Read one ideoFrame at a time
        if(depthFrame.isValid())                  // If the VideoFrame is valid
        {
            vmode = depth.getVideoMode();  // Get the VideoMode Info for this video stream.
                                        // This includes its resolution, fps and stream format.
            const float* imgBuf = (float*)depthFrame.getData();
                                        // PrimeSense gives the IR stream as 16-bit data output
            int h=depthFrame.getHeight();
            int w=depthFrame.getWidth();
            
            cout << "h: " << h << endl;
            cout << "w: " << w << endl;
            //frame.create(h, w, CV_16U); // Create the OpenCV Mat Matrix Class Object
                                        // to receive the IR VideoFrames
            //memcpy(frame.data, imgBuf, h*w*sizeof(uint16_t));
                                        // Copy the ir data from memory imgbuf -> frame.data
                                        // using memcpy (a string.h) function
            //frame.convertTo(frame, CV_8U);
                                        // OpenCV displays 8-bit data (I'm not sure why?)
                                        // So, convert from 16-bit to 8-bit
            //namedWindow("ir", 1);       // Create a named window
            //imshow("ir", frame);        // Show the IR VideoFrame in this window
            //char key = waitKey(10);
            //if(key==27) break;          // Escape key number
        }
    }
}
  
  depth.stop();
  depth.destroy();
  color.stop();
  color.destroy();
  device.close();
  
  // allows open ni to close properly
  //openni::OpenNI::shutdown();
  
  
  std::cout << std::endl 
            << "Everything went well till the end !"
            << std::endl;
  return 1;
}
