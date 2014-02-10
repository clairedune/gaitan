/*
    <one line to give the program's name and a brief idea of what it does.>
    Copyright (C) 2013  <copyright holder> <email>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MATRIX_CONVERSION_H
#define MATRIX_CONVERSION_H
#include <string>
#include <Eigen/Dense>
#include <visp/vpConfig.h>
#include <visp/vpImage.h>
#include <visp/vpRGBa.h>

#include <pcl/common/common_headers.h>


namespace gaitan
{
class Conversion
{
  public:
    static int convert(const vpImage<float>&dmap, vpImage<vpRGBa>&Idmap);
    static int convert(const vpImage<float>&dmap, Eigen::MatrixXf & depthMat);
    static int convert(const Eigen::MatrixXf & depthMat, vpImage<float>&dmap);
    static int convert(const Eigen::MatrixXf & depthMat,Eigen::MatrixXf & point3D,
                                 double &fx,
                                 double &fy,
                                 double &cx,
                                 double &cy);
    static int convert(const Eigen::MatrixXf & point3D,Eigen::MatrixXf & depthMat,
                       const int &height,
                       const int &width,
                       double &fx,
                       double &fy,
                       double &cx,
                       double &cy);
                       
   
   static int convert(const Eigen::MatrixXf & matrix, pcl::PointCloud<pcl::PointXYZRGB>::Ptr & cloud, const int & r, const int & g, const int & b);
   static int convert(const Eigen::MatrixXf & matrix, pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud);
                    
};
}
#endif // Conversion_H
