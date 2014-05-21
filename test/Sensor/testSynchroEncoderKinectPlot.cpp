#include <sys/time.h> // to get time
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <math.h>

#include <Eigen/Dense>

#include <libgaitan/plane.h>
#include <libgaitan/box.h>
#include <libgaitan/conversion.h>
#include <libgaitan/kinect.h>
#include <libgaitan/encoder.h>
#include <libgaitan/synchro.h>
#include <libgaitan/geometry.h>


#include <visp/vpImageIo.h>


#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/pcl_base.h>
#include <cfloat>
#include "pcl/common/impl/common.hpp"
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
//#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <visp/vpConfig.h>
#include <visp/vpRxyzVector.h>
#include <visp/vpConfig.h>
#include <visp/vpPlot.h>


using namespace gaitan;


void homogeneousMatrix (const float& x, const float& y, const float &theta)
{
  
  
}


void userInput (int argc, char** argv, std::string& path, std::string &pattern, int & nbImBegin,int & sampleStep, int & nbImEnd)
{
 
  // get the path name
	if (argc>1){
		path = argv[1];
	}
	else {
//    path ="/home/dune/Documents/data/kinect/essai4/";
      path ="/home/dune/Documents/data/encoder-kinect/essai1";
  }
  
  // get the number of images to treat
  if (argc>2){
    nbImBegin = atoi(argv[2]);
    }
  else  nbImBegin = 0;  
  
  if (argc>3){
    nbImEnd = atoi(argv[3]);
    }
  else  nbImEnd = nbImBegin+100;  
  
  if (argc>4){
    sampleStep = atoi(argv[4]);
    
    }
  else sampleStep=floor((nbImEnd-nbImBegin)/50);  
    
  
}


int cubeVis(boost::shared_ptr<pcl::visualization::PCLVisualizer> & viewer, Box box, float r, float g, float b, std::string name){
  
        viewer->addCube (
      box.getMinX(),
      box.getMaxX(),
      box.getMinY(),
      box.getMaxY(),
      box.getMinZ(),
      box.getMaxZ(), r, g,b, name); 
      
      return 1;
  }

boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
float  a, float  b, float  c, float  d)
{
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
  //                                   cloud->points[cloud->size() - 1], "line");
  //viewer->addSphere (cloud->points[0], 0.2, 0.5, 0.5, 0.0, "sphere");

  //---------------------------------------
  //-----Add shapes at other locations-----
  //---------------------------------------
  pcl::ModelCoefficients coeffs;
  coeffs.values.push_back (a);
  coeffs.values.push_back (b);
  coeffs.values.push_back (c);
  coeffs.values.push_back (d);
 viewer->addPlane (coeffs, "plane");

  return (viewer);
}

// --------------
// -----Main-----
// --------------
int
main (int argc, char** argv)
{


 //------------- INIT -----------------------------------------//
  std::string path, pattern, fullPath;
  int nbIm, nbSampleBegin, nbSampleEnd, sampleStep;
  userInput(argc, argv, path, pattern, nbSampleBegin, sampleStep,nbSampleEnd);
     
   
  // ------------ SET UP For Point Cloud Tools ------------------------------------//
  // segmentation parameters
  float clusterTolerance (0.01); // min dist between two cluster
  int minClusterSize(150);       // min size of a cluster
  int maxClusterSize(50000);     // max size of a cluster
  double confidence(0.02);       // confidence for plane detection
  float leafSize(0.005);          // size of the grid a filtered point cloud
  double distThreshold(0.02);    // min point-to-plane distance when removing points belonging to ground plane

  //-------------Create the kinect from path --------------------------------------//
        
  Kinect * kinect= new Kinect();  
  if(!kinect->loadConfFile(path))
  {
      std::cerr   <<"No configuration file found in "  << path <<". You need to init the kinect first"  << std::endl;
      return -1;
  }
  
  if(!kinect->loadTimeSampling(path))
  {
      std::cerr   <<"No timeFile file found in "  << path << std::endl;
      return -2;
  }
  
  // -------------- Create the encoder from path --------------------//
  Encoder * encoder = new Encoder();
  if(!encoder->load(path))
  {
     std::cerr   <<
        "No odometry file found. You need to compute odometry first"  
        << std::endl;
    return -3;
  }
  
  //------------------ Synchro --------------------------------------//
  Table synchro;  
  Table::synchronize(*(kinect->data), *(encoder->data), synchro);
  
  synchro.print(0,10);
  
  
   // verif   
   if (nbSampleEnd>synchro.getRows()) nbSampleEnd=synchro.getRows();
  
  // ------------------- Visualisation Settings-------------------------------//
  Plane ground;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();  
  
  bool isFirst = true;
  int iteration  =0;
  
  // ---- Plot settings ------//
  // Create a window (700 by 700) at position (100, 200) with two graphics
  vpPlot A(2, 700, 700, 10, 10, "Deambulateur");
  // The first graphic contains 3 curves X,Y,theta 
  A.initGraph(0,3);
  // The second graphic contains 1 curve Y=f(X)
  A.initGraph(1,1);
  // The color of the curve in the first graphic is red
  A.setColor(0,0,vpColor::red);
  // The second curve on the first graph is green
  A.setColor(0,1,vpColor::green);
  // The third curve on the second graph is blue
  A.setColor(0,2,vpColor::blue);
  // The first curve of the second graph is blue
  A.setColor(1,0,vpColor::blue);
  
  
  // Create a window (700 by 700) at position (100, 200) with two graphics
  vpPlot B(3, 700, 700, 700, 10, "Pieds");
  
  // The first graphic contains 3 curves X,Y,Z for foot 1
  B.initGraph(0,3);
  // The second graphic contains 3 curve X,Y,Z for foot 2
  B.initGraph(1,3);
  // The third graphic contains 2 curves XY for the two feet
  B.initGraph(2,2);
  
  // The color of the curve in the first graphic is red
  B.setColor(0,0,vpColor::red);
  // The second curve on the first graph is green
  B.setColor(0,1,vpColor::green);
  // The third curve on the second graph is blue
  B.setColor(0,2,vpColor::blue);
  
  // The color of the curve in the second graphic is red
  B.setColor(1,0,vpColor::red);
  // The second curve on the second graph is green
  B.setColor(1,1,vpColor::green);
  // The third curve on the second graph is blue
  B.setColor(1,2,vpColor::blue);
  
  // The color of the first curve on the third graph is red for left foot
  B.setColor(2,0,vpColor::red);
  // The color of the first curve on the third graph is green for left foot
  B.setColor(2,1,vpColor::green);
  
  
  
  //---------------------------------------------------------//
  int nbSamples(encoder->data->getRows());
  std::cout << "nb samples ::" << nbSamples << std :: endl;
  
  //----- Create the transformation matrix ---- //
  Eigen::MatrixXf wMg(Eigen::MatrixXf::Identity(4,4));
  double x,y,theta;
  std :: cout << synchro.data << std::endl;
 
  // ---------------------Loop----------------------------------------//
  for(int index=nbSampleBegin ; index<=nbSampleEnd ; index+=sampleStep ){
    
    nbIm = synchro.data(index,1);
    // prompt message
    std::cout << "----------------------" << std::endl ;
    std::cout << "  Iteration "           << iteration << std::endl ;
    std::cout << "  Image "               << nbIm << std::endl ;
    std::cout  << " Time "                << std::setprecision(15) << synchro.data(index,0) << std::endl; 
    std::cout << "----------------------" << std::endl ;
    iteration++;
         
    // create the point cloud as an eigen matrix
    Eigen::MatrixXf pt;
    kinect->pointCloud(path,nbIm,pt);
    Eigen::MatrixXf copy (pt);
    std::cout << pt.rows() << "\t Total points" << std::endl;
    Eigen::MatrixXf ptsOut; 
    kinect->limitFov(pt, ptsOut,distThreshold);
    std::cout << pt.rows() << "\t Total points in field of view" << std::endl;
    if(pt.rows()<1) return -2;
      
    //----- init the kinect pose wrt the ground only for the first image
    if(isFirst)
    {  
      ground = kinect->extrinsicCalibration(pt,confidence,leafSize);   
      isFirst=false;
    }
    // select the points that are not on the ground
    Eigen::MatrixXf ptsGround(pt),ptsFeetNWheels(1,3) ;
    kinect->clearGroundPoints(ptsGround,ptsFeetNWheels,distThreshold,&ground);  
    
    // change the points frame
    Eigen::MatrixXf gPtsFeetNWheels  = kinect->changeFrame(ptsFeetNWheels);     
  
  
    // Clear forbidden zone
    Eigen::MatrixXf gPtsWheels(gPtsFeetNWheels), gPtsFeet(1,3); 
    kinect->clearForbiddenZone(gPtsWheels, gPtsFeet, distThreshold);
    
    
    
   
       
             
    //---------Translation--------//
    
    x = synchro.data(index,2);
    //std::cout << "x: "<< x << std::endl;
    y = synchro.data(index,3);
    //std::cout << "y: "<< y << std::endl;
    theta = synchro.data(index,4);
    //theta = 0;//synchro.data(index,4);
    //std::cout << "theta: "<< theta << std::endl;
    
    Geometry::homogeneousMatrix(x,y,theta, wMg);
    //std::cout << "Transformation wMg:\n" << wMg << std::endl;  
    
    Eigen::MatrixXf wPtsFeet=kinect->changeFrame(gPtsFeet,wMg);         
    Eigen::MatrixXf wPtsWheels=kinect->changeFrame(gPtsWheels,wMg); 
    
    
    //------------------------------------------------------------//
    //
    //  Detect the feet cluster and trace bounding boxes
    //
    //------------------------------------------------------------//     
    std::vector<Box> feetBoundingBoxes;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFeetFiltered (new pcl::PointCloud<pcl::PointXYZ>);
    std::vector<pcl::PointIndices> clusterIndices;
    kinect->detectClusters(wPtsFeet, cloudFeetFiltered,clusterIndices, clusterTolerance, minClusterSize,maxClusterSize,leafSize);
    kinect->clusterBoundingBoxes(wPtsFeet, cloudFeetFiltered,clusterIndices,feetBoundingBoxes);
          
    // Display
         
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Conversion::convert(wPtsFeet,colorCloud, 0, 255,0);
    Conversion::convert(wPtsWheels,colorCloud, 255, 0,0);
    
    // viewer remove all points, ok even if there is no point inside
    viewer->removeAllPointClouds();
    viewer->removeAllShapes();
    
    // add the points in the viewer
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colorCloud);
    viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud, rgb,"sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud"); 
  
  
    // display the boxes corresponding to the forbidden zone
    /*for (int i =0; i<kinect->forbiddenZone.size();i++) {
       std::string tmp = "cube-%d";
       char buf[100];
       sprintf(buf,tmp.c_str(),i);
       std::string cubeName(buf);      
       cubeVis(viewer, kinect->forbiddenZone[i], 1.0, 0.0,0.0,cubeName);
    }*/
    float xb,yb,zb;
    bool verbose = true;
    for (int i =0; i<feetBoundingBoxes.size();i++) {
       feetBoundingBoxes[i].center (xb,yb,zb);

      if (verbose){
       std::cout << " Box "<< i << " :" << std::endl;
       feetBoundingBoxes[i].print();
       std::cout << "x : "<< xb << "\t" ;
       std::cout << "y : "<< yb << "\t" ;
       std::cout << "z : "<< zb << "\t" << std::endl;
       std::cout << "volume : " << feetBoundingBoxes[i].volume() << std::endl;
       }
     
       std::string tmp = "foot-%d";
       char buf[100];
       sprintf(buf,tmp.c_str(),i);
       std::string footName(buf);      
       cubeVis(viewer, feetBoundingBoxes[i], 0.0, 1.0,0.0,footName);
    }
    
    
    viewer->spinOnce ();
    
    //--------------------------------------------//
     A.plot(0,0,index,x);
     A.plot(0,1,index,y);
     A.plot(0,2,index,theta);  
     A.plot(1,0,x,y);
     
     
    for (int i =0; i<feetBoundingBoxes.size();i++) 
    {
       feetBoundingBoxes[i].center (xb,yb,zb);
       
       if (i<2){
       B.plot(i,0,index,xb);
       B.plot(i,1,index,yb);
       B.plot(i,2,index,zb);  
     }
/*       B.plot(2,i,xb,yb);*/
    }
  }
      
  char a;
  std:: cin >> a;
  
  return 0;
     
  
    
   }



