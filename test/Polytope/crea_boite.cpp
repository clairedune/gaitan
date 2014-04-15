#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <iomanip>
#include <math.h>
#include <Eigen/Dense>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

using namespace std;

int load(string filename,Eigen::MatrixXd & data){
  ifstream file(filename.c_str(), ios::in);
  if(file)
    {
      //count lines
      int nbLine=0;
      string line;
      while(getline(file, line))
        nbLine++;     
     
      // on revient au début du fichier
      file.clear();
      file.seekg(0,ios::beg);

      int nbCol = 2;
      data.resize(nbLine,nbCol);
      double x;
      for(int i=0;i<nbLine;i++){
          for(int j=0;j<nbCol;j++){
          file >> x;
          data(i,j)=x;     
          }
      }
      file.close();
    }
  else
    {
      cerr << "impossible d'ouvrir le fichier"<<endl;
      return 0;
    }
   return 1;
}

int getbox (Eigen::MatrixXd data,int i,double & xmin,double & xmax,double & ymin, double & ymax, double & zmin, double & zmax) {
	xmin = data(i,0);
	xmax = data(i,1);
	ymin = data(i+1,0);
	ymax = data(i+1,1);
	zmin = data(i+2,0);
	zmax = data(i+2,1);
	return 1;
}

int main(int argc, char** argv) {
	  
  // ---- INIT ------ //
  string filename;
  if (argc > 1) 
  {
		filename = argv[1]; 
  }
  else 
  { 
    cerr << "erreur " << argv[0] << " filename" << endl ;
		return -1; 
  }
  cout << filename << endl;


 //----- LOAD --------//
  
  Eigen::MatrixXd data(1,1);
	load(filename,data);
	//cout << data << endl;

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (1.0, 1.0, 1.0);
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();

	double xmin(0),xmax(0),ymin(0),ymax(0),zmin(0),zmax(0);

  // nb Boxst
  int nbBox(data.rows()/3);
  
  int i=0;
  for (i =0 ; i< nbBox ; i++)
  {
    std::string tmp = "foot-%d";
    char buf[100];
    sprintf(buf,tmp.c_str(),i);
    std::string boxName(buf);      
    getbox (data,i*3,xmin,xmax,ymin,ymax,zmin,zmax);
	  viewer->addCube(xmin,xmax,ymin,ymax,zmin,zmax, 1.0, 0.0,0.0,boxName );
    
  }
  int iter=0;
  std::string dirToSave("/home/dune/rush/");
  std::string fileToSave;
  
	while (!viewer->wasStopped ())
  {
    iter++;
    stringstream spad5;
    spad5 <<"snapshot_"<< std::setw(4) << std::setfill('0') << iter << ".png";
    fileToSave=dirToSave+ spad5.str();
    std::cout<< fileToSave << std::endl;
    
    
    viewer->spinOnce ();
    viewer->saveScreenshot(fileToSave);

    boost::this_thread::sleep (boost::posix_time::microseconds (1000));
    

   // viewer->saveScreenshot("test.png");
    }
	return 1;
}
