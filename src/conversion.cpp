#include <libgaitan/conversion.h>
#include <Eigen/Dense>
#include <visp/vpConfig.h>



#include <pcl/common/common_headers.h>

namespace gaitan
{    

int Conversion::convert(const vpImage<float>&dmap, vpImage<vpRGBa>&Idmap)
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
              Idmap[i][j].R = 0;
              Idmap[i][j].G = 0;
              Idmap[i][j].B = 0;
            }
          }
        }
        return 1;
}

int Conversion::convert(const vpImage<float>&dmap, Eigen::MatrixXf & depthMat)
{
      int height = dmap.getHeight();
      int width  = dmap.getWidth();
      // i <-> height and j<->width
      depthMat.resize(height,width);
      for(int i = 0 ; i< height ; i++){
       for(int j=0 ; j< width ; j++){
              depthMat(i,j) = dmap[i][j];
        }
      }
      
      return 1;
}

int Conversion::convert(const vpImage<float> & dmap, Eigen::MatrixXf & point3D, 
double fx, double fy, double cx, double cy)
{
      int height = dmap.getHeight();
      int width  = dmap.getWidth();
      point3D.resize(height*width,3);
      int index=0;
      for(int i = 0 ; i< height ; i++){
       for(int j=0 ; j< width ; j++){
            float z =dmap[i][j];
           if (fabs(z + 1.f) > std::numeric_limits<float>::epsilon() & fabs(z) !=1){
            point3D(index,2) = z;
            point3D(index,0) = (i-cx)*point3D(index,2)/fx; 
            point3D(index,1) = (j-cy)*point3D(index,2)/fy;
            index++;
          }
        }
      }
      
      return 1;
  
}




int Conversion::convert(const Eigen::MatrixXf & depthMat, vpImage<float>&dmap)
{
      int height = depthMat.rows();
      int width  = depthMat.cols();
      dmap.resize(height, width);
      for(int i = 0 ; i< height ; i++){
       for(int j=0 ; j< width ; j++){
              dmap[i][j]=depthMat(i,j);
        }
      }
      
      return 1;
}


int Conversion::convert(const Eigen::MatrixXf & depthMat,
                                 Eigen::MatrixXf & point3D,
                                 double &fx,
                                 double &fy,
                                 double &cx,
                                 double &cy)
{
      point3D.resize(depthMat.rows()*depthMat.cols(),3);
      int index(0);
      for (int i=0; i<depthMat.rows();i++)
        for (int j=0 ; j<depthMat.cols();j++)
        {
          if (fabs(depthMat(i,j) + 1.f) > std::numeric_limits<float>::epsilon() & fabs(depthMat(i,j)) !=1){
            point3D(index,2) = depthMat(i,j);
            point3D(index,0) = (i-cx)*point3D(index,2)/fx; 
            point3D(index,1) = (j-cy)*point3D(index,2)/fy;
            index++;
          }
        }
        
        return 1;
}     

int Conversion::convert(const Eigen::MatrixXf & depthMat,
                              pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud,
                              double &fx,
                              double &fy,
                              double &cx,
                              double &cy)
{
      int index(0);
      for (int i=0; i<depthMat.rows();i++)
        for (int j=0 ; j<depthMat.cols();j++)
        {
          if (fabs(depthMat(i,j) + 1.f) > std::numeric_limits<float>::epsilon()){
            pcl::PointXYZ basic_point;
            basic_point.x = depthMat(i,j);
            basic_point.y = (i-cx)*depthMat(i,j)/fx;
            basic_point.z = (j-cy)*depthMat(i,j)/fy;
            cloud->push_back(basic_point);
            index++;
          }
        }
        return 1;
}     



int Conversion::convert(const Eigen::MatrixXf & point3D,
                        Eigen::MatrixXf & depthMat,
                                 const int &height,
                                 const int &width,
                                 double &fx,
                                 double &fy,
                                 double &cx,
                                 double &cy)
{
      if(point3D.rows()==0)
        return 0;
        
      depthMat.resize(height,width);
      for (int i=0; i<depthMat.rows();i++)
        for (int j=0 ; j<depthMat.cols();j++)
          depthMat(i,j)=-1;
      
      
      for (int index=0; index<point3D.rows();index++)
        {
          float val = point3D(index,2);//*1000;
          //if(val>0){
          int   i   = round(fx*point3D(index,0)/point3D(index,2)+cx);
          int   j   = round(fy*point3D(index,1)/point3D(index,2)+cy);
          depthMat(i,j)=val;
          //}
          index++;
        }
        return 1;
}  



int Conversion::convert(const Eigen::MatrixXf & matrix, 
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, 
                        const int & r, const int & g, const int & b)
{
 for (int i=0; i<matrix.rows();i++){
            pcl::PointXYZRGB basic_point;
            basic_point.x = matrix(i,0);
            basic_point.y = matrix(i,1);
            basic_point.z = matrix(i,2);
            basic_point.r = r;
            basic_point.g = g;
            basic_point.b = b;
            cloud->push_back(basic_point);
      }
      return 1;
}

int Conversion::convert(const Eigen::MatrixXf & matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud){
 for (int i=0; i<matrix.rows();i++){
            pcl::PointXYZ basic_point;
            basic_point.x = matrix(i,0);
            basic_point.y = matrix(i,1);
            basic_point.z = matrix(i,2);
            cloud->push_back(basic_point);
      }
 return 1;
}

int Conversion::convert(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, Eigen::MatrixXf & matrix){
  
 matrix.resize(cloud->points.size(), 3); 
 for (int i=0; i<cloud->points.size();i++){
            pcl::PointXYZ basic_point(cloud->points[i]);
            matrix(i,0)=basic_point.x ;
            matrix(i,1)=basic_point.y ;
            matrix(i,2)=basic_point.z ;
      }
 return 1;
}


int Conversion::convert(const pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & colorCloud, 
                       const int & r, const int & g, const int & b){
       for (int i=0; i<cloud->points.size ();i++){
            pcl::PointXYZRGB point;
            point.x = cloud->points[i].x;
            point.y = cloud->points[i].y;
            point.z = cloud->points[i].z;
            point.r = r;
            point.g = g;
            point.b = b;
            colorCloud->push_back(point);
      }
      return 1;
  
  }



}//end namespace   
