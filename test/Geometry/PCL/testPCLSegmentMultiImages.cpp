/* \author Geoffrey Biggs */


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

#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
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


/*!
 *  align the camera frame with the ground
 */
Eigen::MatrixXf changeFrame(const Eigen::MatrixXf & oP,const Eigen::Matrix4f & wMo)
{
   Eigen::MatrixXf wP(oP.rows(),oP.cols());
   for(int i = 0 ; i < oP.rows(); i++ ){
      // create an homogenous point from 3D coordinate
      Eigen::Vector4f oPi;
      oPi(0) = oP(i,0);
      oPi(1) = oP(i,1);
      oPi(2) = oP(i,2);
      oPi(3) = 1;
      // change frame
      Eigen::Vector4f wPi  = wMo*oPi;
      // insert in the matrix
      wP(i,0) = wPi(0);
      wP(i,1) = wPi(1);
      wP(i,2) = wPi(2);
   }
   return wP;
}




boost::shared_ptr<pcl::visualization::PCLVisualizer> shapesVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud,
float & a, float & b, float & c, float & d, std::string &id)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, id);
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3,id);
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
  
 // coeffs.values.push_back (-0.0478);
 // coeffs.values.push_back (0.3893);
 // coeffs.values.push_back (0.7084);
 // coeffs.values.push_back (-0.5867);
  viewer->addPlane (coeffs, "plane");

  return (viewer);
}






// --------------
// -----Main-----
// -------------
int
main (int argc, char** argv)
{
  //data folder path
  std::string path;
  //data filename
  std::string filename("depth_%07d.pfm"), fullPath;
  //number of images to treat
  int nbIm;
 
  // get the path name
	if (argc>1){
		path = argv[1];
	}
	else {
    path ="/home/dune/Documents/data/kinect/essai1";
  }
  
  // get the number of images to treat
  if (argc>2){
    nbIm = atoi(argv[2]);
    }
  else  nbIm = 50;
  
  
	
	try {
      
      
      // create the visp image to read the visp file          
      int width(640), height(480);
      vpImage<float> dmap(height,width);//for medium resolution

   
      // create the pcl viewer
      boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      viewer->setBackgroundColor (0, 0, 0);
      viewer->addCoordinateSystem (1.0);
      viewer->initCameraParameters ();   
   

      // ground plane : assumed to be the same for all images
      Plane plane, gPlane;      
    
      // tranformation matrix to align the frame with the matrix
      Eigen::Matrix4f gMk = MatrixXf::Identity(4,4);
      
      for(int index=0; index<nbIm ; index++){
      
         std::cout << "----------------------" << std::endl ;
         std::cout << "Image " << index << std::endl ;
         std::cout << "----------------------" << std::endl ;
        
        // build the image name
        std::string tmp = path + "/" + filename;
        char buf[100];
        sprintf(buf,tmp.c_str(),index);
        std::string fullPath(buf);
        std::cout << fullPath << std::endl;
        // read the image
        try{
            vpImageIo::readPFM(dmap,fullPath.c_str());
        }
        catch(...){
               std::cout << "Catch an exception when reading image " << filename << std::endl;
        }
        
        // copy the matrix in an eigen matrix
        Eigen::MatrixXf depthMap, pointCloud;
        Conversion::convert(dmap, depthMap);
        double fx(525.0), fy(525.0), cx(319.05), cy(239.5);
        Conversion::convert(depthMap,pointCloud,fx,fy,cx,cy);

        // find the coeff of the main plane
        Eigen::MatrixXf ptsIn(pointCloud), ptsOut(3,1) ;
        double confidence(0.02);
        pcl::PointCloud<pcl::PointXYZ>::Ptr simpleCloud(new pcl::PointCloud<pcl::PointXYZ>);
        Conversion::convert(pointCloud,simpleCloud);
        
        // find the plane parameters only for the first image
      //  if(index==0){
              plane.findParameters(simpleCloud,confidence);
              gMk = plane.computeTransformation();
              plane.print();
              gPlane = plane;
              gPlane.changeFrame(gMk);
        //}
      
        // divide the point cloud into two clouds inliers that are in the plane
        // and outliers that are the points of interest
        plane.inlierSelection(ptsIn, ptsOut, confidence);
        
        Eigen::MatrixXf gPtsIn  = changeFrame(ptsIn,gMk);
        Eigen::MatrixXf gPtsOut = changeFrame(ptsOut,gMk);
      
        //populate cloud for visualisation    
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorCloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        //Conversion::convert(gPtsIn,colorCloud, 255, 0, 0);
        //Conversion::convert(gPtsOut,colorCloud, 0, 255,0);
      
        // keep only the point that are not on ground
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Conversion::convert(gPtsOut,cloud);
        std::cout << "PointCloud before filtering has: " << cloud->points.size ()  << " data points." << std::endl;  
      
        // Create the filtering object: downsample the dataset using a leaf size of 5mm
        pcl::VoxelGrid<pcl::PointXYZ> vg;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZ>);
        vg.setInputCloud (cloud);
        vg.setLeafSize (0.01f, 0.01f, 0.01f);
        vg.filter (*cloudFiltered);
        std::cout << "PointCloud after filtering has: " << cloudFiltered->points.size ()  << " data points." << std::endl; 

      
        // Creating the KdTree object for the search method of the extraction 
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud (cloudFiltered);     
        std::vector<pcl::PointIndices> clusterIndices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance (0.02); // 2cm
        ec.setMinClusterSize (500);
        ec.setMaxClusterSize (50000);
        ec.setSearchMethod (tree);
        ec.setInputCloud (cloudFiltered);
        ec.extract (clusterIndices);

        // TODO TESTER S'il n'y a pas de cluster ... 
  
  
        // TODO Stocker que les deux plus grands clusters

        // Creating the Clusters
        int j(0);
        for (std::vector<pcl::PointIndices>::const_iterator it = clusterIndices.begin (); it != clusterIndices.end (); ++it)
        {
          pcl::PointCloud<pcl::PointXYZ>::Ptr cloudCluster (new pcl::PointCloud<pcl::PointXYZ>);
          for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
           cloudCluster->points.push_back (cloudFiltered->points[*pit]); 
          cloudCluster->width = cloudCluster->points.size ();
          cloudCluster->height = 1;
          cloudCluster->is_dense = true;
       
          Conversion::convert(cloudCluster,colorCloud, j*100, j*100,255);
          std::cout << "PointCloud representing the Cluster: " << cloudCluster->points.size () << " data points." << std::endl;
          j++;
        }
        std::cout << "There are " << j << " clusters " << endl;
      
        // viewer
        // create the id of the current cloud and of the previous one
        std::stringstream out;
        out <<  "cloud_" <<index;        
        std::string cloudId(out.str());
        
        std::stringstream outPrec;
        int indexPrec = index-1;
        outPrec << "cloud_"<<indexPrec;
        std::string cloudIdPrec(outPrec.str());
        
        std::cout << "Current Cloud id : " << cloudId << std::endl;
        std::cout << "Prec Cloud id : " << cloudIdPrec << std::endl;
        
        
        if (index>=1) viewer->removePointCloud(cloudIdPrec); 
         
        std:: cout << " Insertion des points" << std::endl;
        pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(colorCloud);
        viewer->addPointCloud<pcl::PointXYZRGB> (colorCloud, rgb, cloudId);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, cloudId);

        Eigen::VectorXf param = gPlane.getParameters(); 
        pcl::ModelCoefficients coeffs;
        coeffs.values.push_back (param(0));
        coeffs.values.push_back (param(1));
        coeffs.values.push_back (param(2));
        coeffs.values.push_back (param(3));
        std::stringstream planeIdStream;
        planeIdStream <<  "plane_" <<index;        
        std::string planeId(planeIdStream.str());
        viewer->addPlane (coeffs, planeId);  
      
      int elapse(0);
      //while (!viewer->wasStopped () & elapse < 10)
      while (elapse < 10)
      {
        viewer->spinOnce (100);
        boost::this_thread::sleep (boost::posix_time::microseconds (1000000));
        elapse ++;
        std::cout << elapse << std::endl;
      }
    }
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
