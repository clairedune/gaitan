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
#include <libgaitan/foot.h>
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
  
  
int initGraph(vpPlot & B)
{
    
  // The first graphic contains 3 curves X,Y,Z for foot 1
  B.initGraph(0,3);
  // The second graphic contains 3 curve X,Y,Z for foot 2
  B.initGraph(1,3);
  // The third graphic contains 2 curves XY for the two feet and the deamb
  B.initGraph(2,3);
  
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
  // The color of the first curve on the third graph is green for left foot
  B.setColor(2,2,vpColor::black);
    
}  


int pointCloud(Kinect*kinect, std::string path, 
                int nbIm, bool& isFirst,double distThreshold, 
                double leafSize, double confidence, Plane &ground,
                Eigen::MatrixXf &gPtsWheels, Eigen::MatrixXf &gPtsFeet )
{
    Eigen::MatrixXf pt;
    kinect->pointCloud(path,nbIm,pt);
   
    Eigen::MatrixXf ptsOut; 
    std::cout << " nb point before the FOV " << pt.rows()<< std::endl;

    kinect->limitFov(pt, ptsOut,distThreshold);
      if(pt.rows()<1) return -2;
      
      
      
    
    std::cout << " nb point after the FOV " << pt.rows()<< std::endl;
    std::cout << " nb point rejected after the FOV " << ptsOut.rows()<< std::endl;

      
    // init the kinect pose wrt the ground only for the first image
    if(isFirst){  
      ground = kinect->extrinsicCalibration(pt,confidence,leafSize);   
      isFirst=false;
    }
    
    // select the points that are not on the groundu
    Eigen::MatrixXf ptsGround(pt),ptsFeetNWheels(1,3) ;
    kinect->clearGroundPoints(ptsGround,ptsFeetNWheels,distThreshold,&ground);  
    
    // change the points frame
    Eigen::MatrixXf gPtsFeetNWheels  = kinect->changeFrame(ptsFeetNWheels);     
  
    // Clear forbidden zone
    gPtsWheels = gPtsFeetNWheels; 
    kinect->clearForbiddenZone(gPtsWheels, gPtsFeet, distThreshold);  
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
  int minClusterSize(400);       // min size of a cluster
  int maxClusterSize(50000);     // max size of a cluster
  double confidence(0.02);       // confidence for plane detection
  float leafSize(0.005);          // size of the grid a filtered point cloud
  double distThreshold(0.02);    // min point-to-plane distance when removing points belonging to ground plane

  //-------------Create the kinect from path --------------------------------------//
  Kinect * kinect= new Kinect(594.0,591.0,339.05, 247.5);  
  if(!kinect->loadConfFile(path)){
      std::cerr   <<"No configuration file found in "  << path <<". You need to init the kinect first"  << std::endl;
      return -1;
  }
  
  if(!kinect->loadTimeSampling(path)){
      std::cerr   <<"No timeFile file found in "  << path <<". You need to do the acquisition again." << std::endl;
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
  
  // ------------------- Visualisation Settings-------------------------------//
  Plane ground;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();  
    
  // ---- Plot settings ------//
  // Create a window (700 by 700) at position (100, 200) with two graphics
  vpPlot B(3, 700, 700, 700, 10, "Pieds");
  initGraph(B);

 
    
  // ---------------------Loop----------------------------------------//
  // Create the transformation matrix
  Eigen::MatrixXf wMg(Eigen::MatrixXf::Identity(4,4));
  double x,y,theta;
  
   // Create the two feet
   Foot foot1,foot2, footcurrent; // at the beginning we do not know which foot is which

   // init variable
   bool isFirst = true;
   int iteration  =0;
  
  
   for(int index=nbSampleBegin ; index<=nbSampleEnd ; index+=sampleStep ){
    nbIm = synchro.data(index,1);
    double currtime = synchro.data(index,0);
    // prompt message
    std::cout << "----------------------" << std::endl ;
    std::cout << "  Iteration "           << iteration << std::endl ;
    std::cout << "  Image "               << nbIm << std::endl ;
    std::cout  << " Time "                << std::setprecision(15) << currtime << std::endl; 
    std::cout << "----------------------" << std::endl ;
    iteration++;
         
         
    // create the point cloud as an eigen matrix                    
    Eigen::MatrixXf gPtsWheels,gPtsFeet,gPtsFeetout;
    pointCloud(kinect,path,nbIm,isFirst,distThreshold, leafSize, confidence, ground,gPtsWheels, gPtsFeet );

    // limit the leg
    Box limitLeg(-0.6,0.1,-0.4,0.4,0,0.1);
    limitLeg.inlierSelection(  gPtsFeet, 
                      gPtsFeetout, 
                      distThreshold);

    //---------odometry integration-----//
    x = synchro.data(index,2);
    //std::cout << "x: "<< x << std::endl;
    y = synchro.data(index,3);
    //std::cout << "y: "<< y << std::endl;
    theta = synchro.data(index,4);
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
    
    
    // just for display
    float xb,yb,zb;
    bool verbose = true;
    for (int i =0; i<feetBoundingBoxes.size();i++) {
      
      if (verbose){
       std::cout << " Box "<< i << " :" << std::endl;
       feetBoundingBoxes[i].print();
       
       feetBoundingBoxes[i].center(xb,yb,zb);
       std::cout << "----------------------------------------" <<std::endl;
       std::cout << "center computed using the center function:\n";
       std::cout << "X center : "<< xb << "\t" ;
       std::cout << "Y center : "<< yb << "\t" ;
       std::cout << "Z center : "<< zb << "\t" << std::endl;
       std::cout << "volume : " << feetBoundingBoxes[i].volume() << std::endl;
      
       
       }
     
      std::string tmp = "foot-%d";
       char buf[100];
       sprintf(buf,tmp.c_str(),i);
       std::string footName(buf);      
       cubeVis(viewer, feetBoundingBoxes[i], 1.0, 1.0,1.0,footName);
     
      // std::string tmp = "foot-%d";
      // char buf[100];
      // sprintf(buf,tmp.c_str(),i);
      // std::string footName(buf);      
      // cubeVis(viewer, feetBoundingBoxes[i], 0.0, 1.0,0.0,footName);
    }
     
    //----------------------------------------------------------//
    // Associate clusters to feet
    //----------------------------------------------------------//
    
    Foot::footSide labelBox1(Foot::UNKNOWN),labelBox2(Foot::UNKNOWN);
    
    // CASE 1 : only one foot
    if(feetBoundingBoxes.size()==1)
    {
      
      std::cout << "------- 1 Foot ---------- "<< std::endl;
      
      if(verbose)
       std::cout << "Foot info for iteration "<< index << ":\n" <<
       "there is only 1 tfoot" << std::endl;  
       
       // set it as the current foot measure
       footcurrent.update(feetBoundingBoxes[0]);
       std::cout << "Foot caracteristics: "<< std::endl;
       footcurrent.print();
       feetBoundingBoxes[0].center(xb,yb,zb);
       std::cout << "Foot car xb : " << xb 
                 << " Foor car yb : " << yb
                 << " Foor car zb : " << zb << std::endl;
                 
       
       // find which foot it is
       int side = footcurrent.whichFoot(foot1,foot2);
       // rk: at the first step, the two feet are the same, then, one is 
       // chosen by chance
       
       if (verbose)
          {          
            std::cout << "this is the foot number : " <<
            side << std::endl;
            
            std::cout << "indeed the foot car where : "<< std::endl;
            foot1.print();
            foot2.print();
          
          }
          
          
       // foot info update   
       if(side==1)
       {
        foot1.update(footcurrent);
        labelBox1 = foot1.getLabel();           
       }  
       else
       {
        foot2.update(footcurrent);
        labelBox1 = foot2.getLabel(); 
       }
       
       
      //display
     
        if(labelBox1==Foot::LEFT)
        {
            cubeVis(viewer, feetBoundingBoxes[0], 0.0, 1.0,1.0,"foot left");
        }
        else if(labelBox1==Foot::RIGHT)
        {
          cubeVis(viewer, feetBoundingBoxes[0], 1.0, 0.0,1.0, "foot right");
        }
       
       
       
       
    }
    else if(feetBoundingBoxes.size()==2)
    {
        std::cout << "------- 2 Feet ---------- "<< std::endl;
       
       // set it as the current foot measure
       footcurrent.update(feetBoundingBoxes[0]);
       
       // find which foot it is
       int side = footcurrent.whichFoot(foot1,foot2);
       // rk: at the first step, the two feet are the same, then, one is 
       // chosen by chance
       
       if (verbose)
          std::cout << "the first boxe is the foot number : " <<
          side << std::endl;
          
       // foot info update   
       if(side==1)
       {
        foot1.update(footcurrent);
        // set it as the current foot measure
        foot2.update(feetBoundingBoxes[1]);
        labelBox1=foot1.getLabel();  
        labelBox2=foot2.getLabel();  
       }
       else
       {
        foot2.update(footcurrent);
        foot1.update(feetBoundingBoxes[1]);
        labelBox1=foot2.getLabel(); 
        labelBox2=foot1.getLabel(); 
        
       } 
         
       Foot::autoLabel(foot1,foot2);  
         
        //display
        if(labelBox1==Foot::LEFT)
        {
            cubeVis(viewer, feetBoundingBoxes[0], 0.0, 1.0,1.0,"foot 1 Left");
        }
        else if(labelBox1==Foot::RIGHT)
        {
          cubeVis(viewer, feetBoundingBoxes[0], 1.0, 0.0,1.0,"foot 1 right");
        }  
        
        if(labelBox2==Foot::LEFT)
        {
            cubeVis(viewer, feetBoundingBoxes[1], 0.0, 1.0,1.0,"foot 2 left");
        }
        else if(labelBox2==Foot::RIGHT)
        {
          cubeVis(viewer, feetBoundingBoxes[1], 1.0, 0.0,1.0,"foot 2 right");
        }  
         
         
    }
    else 
    {
      std::cerr << "Plus de deux pieds " << std::endl;  
    }
    
    foot1.print();  
    foot2.print();  
    
    

    
    
    viewer->spinOnce ();
    
    //--------------------------------------------//
     //A.plot(0,0,index,x);
     //A.plot(0,1,index,y);
     //A.plot(0,2,index,theta);  
     //A.plot(1,0,x,y);
     
     
    //for (int i =0; i<feetBoundingBoxes.size();i++) 
    //{
     //  feetBoundingBoxes[i].center (xb,yb,zb);
    
    // Foot 1
      double x1, y1, z1;
      x1 = foot1.getToeX();
      y1 = foot1.getToeY();
      z1 = foot1.getToeZ();
      B.plot(0,0,index,x1);
      B.plot(0,1,index,y1);
      B.plot(0,2,index,z1);  
      
    // Foot 2
      double x2,y2,z2;
      x2 = foot2.getToeX();
      y2 = foot2.getToeY();
      z2 = foot2.getToeZ();
      B.plot(1,0,index,x2);
      B.plot(1,1,index,y2);
      B.plot(1,2,index,z2);  
      
    // TOP VIEW
      B.plot(2,0,x1,y1); // foot 1  
      B.plot(2,1,x2,y2); // foot 2  
      B.plot(2,2,x,y);   // deambulateur/walker
      
      
    // }
/*       B.plot(2,i,xb,yb);*/
    //}
  }
      
  char a;
  std:: cin >> a;
  
  return 0;
     
  
    
   }



