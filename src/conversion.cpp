#include <libgaitan/conversion.h>
#include <Eigen/Dense>
#include <visp/vpConfig.h>


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
          if (fabs(depthMat(i,j) + 1.f) > std::numeric_limits<float>::epsilon()){
            point3D(index,2) = depthMat(i,j)/1000; //to convert into meters 
            point3D(index,0) = (i-cx)*point3D(index,2)/fx; 
            point3D(index,1) = (j-cy)*point3D(index,2)/fy;
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
          float val = point3D(index,2)*1000;
          int   i   = round(fx*point3D(index,0)/point3D(index,2)+cx);
          int   j   = round(fy*point3D(index,1)/point3D(index,2)+cy);
          depthMat(i,j)=val;
          index++;
        }
        return 1;
}  

}//end namespace   
