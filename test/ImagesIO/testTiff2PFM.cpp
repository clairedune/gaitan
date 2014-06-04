

#include <visp/vpConfig.h>

#include <iostream>
#include <boost/filesystem.hpp>
#include <sys/time.h> // to get time
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <string>
#include <iomanip>
#include <math.h>


#include <pcl/pcl_base.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>

#include <pcl/io/vtk_lib_io.h>
#include <vtkSmartPointer.h>
#include <vtkImageViewer2.h>
#include <vtkTIFFReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>


#include <visp/vpConfig.h>
#if (defined (VISP_HAVE_X11) || defined(VISP_HAVE_GTK) || defined(VISP_HAVE_OPENCV) || defined(VISP_HAVE_GDI))
#include <visp/vpImage.h>
#include <visp/vpDisplayX.h>
#include <visp/vpDisplayGTK.h>
#include <visp/vpDisplayOpenCV.h>
#include <visp/vpDisplayGDI.h>
#include <visp/vpKinect.h>
#include <visp/vpTime.h>
#include <visp/vpImageIo.h>

using namespace pcl;

void processAndSave( vtkSmartPointer<vtkImageData> depth_data,
                     std::string time,
                     bool use_output_path,
                     std::string output_path)
{

  // Retrieve the entries from the image data and copy them into the output RGB cloud
  int depth_components = depth_data->GetNumberOfScalarComponents();
  //std::cout << "Depth comp:" << depth_components << std::endl;

  if(depth_components != 1)
  {
    std::cout << "Depth image doesn't have a single component, proceed with next image" << std::endl;
    return;
  }

  int depth_dimensions[3];
  depth_data->GetDimensions (depth_dimensions);
  std::cout << "Depth dim1: " << depth_dimensions[0] << " dim2: " << depth_dimensions[1] << " dim3: " << depth_dimensions[2] << std::endl;

  float bad_point = std::numeric_limits<float>::quiet_NaN ();

  vpImage<float> dmap(depth_dimensions[1],depth_dimensions[0]);
  
  for(int v = 0; v < depth_dimensions[1]; v++)
  {
    for(int u = 0; u < depth_dimensions[0]; u++)
    {
      float d = depth_data->GetScalarComponentAsFloat(u,v,0,0);

      if(d != 0 && !pcl_isnan(d) && !pcl_isnan(d))
      {
        dmap[v][u] = d * 0.001f;
      }
      else
      {
        dmap[v][u] = bad_point;
      }

    } // for u
  } // for v

  std::stringstream ss;

    if(use_output_path)
      ss << output_path << "/depth_" << time << ".pfm";
    else
      ss << "./depth_" << time << ".pfm";
      
    try{
          std::string filename(ss.str());
          vpImageIo::writePFM(dmap,filename.c_str());
        }
        catch(...){
              std::cout << "Catch an exception when writing image " << ss << std::endl;
        }  

    std::cout << "Saved depth" << time << ".tiff to" <<ss.str()<< std::endl;
    ss.str(""); //empty
    return;
}

void print_usage(void)
{
  PCL_INFO("usage: convert -depth <depth_path> -out <output_path> options\n");
  PCL_INFO("This program converts rgb and depth tiff files to pcd files");
  PCL_INFO("Options:\n");
  PCL_INFO("\t -v \t Set verbose output\n");
  PCL_INFO("\t -h \t This help\n");
}

int main(int argc, char ** argv)
{
  // TODO: adjust these
  if(argc < 3)
  {
    print_usage();
    exit(-1);
  }

  bool verbose = 0;
  pcl::console::parse_argument (argc, argv, "-v", verbose);

  std::string depth_path_, output_path_;

  if(pcl::console::parse_argument (argc, argv, "-depth", depth_path_) == 0)
  {
    PCL_ERROR("No Depth Path given\n");
    print_usage();
    exit(-1);
  }
  bool use_output_path = false;
  if(pcl::console::parse_argument (argc, argv, "-out", output_path_) == 0)
  {
    PCL_ERROR("No Output Path given\n");
    print_usage();
    exit(-1);
  }
  else
  {
    use_output_path = true;
  }

  float focal_length = 525.0;
  pcl::console::parse_argument (argc, argv, "-f", focal_length);

  if(verbose)
    PCL_INFO ("Creating Depth Tiff List\n");

  std::vector<std::string> tiff_depth_files;
  std::vector<boost::filesystem::path> tiff_depth_paths;
  boost::filesystem::directory_iterator end_itr;

  if(boost::filesystem::is_directory(depth_path_))
  {
    for (boost::filesystem::directory_iterator itr(depth_path_); itr != end_itr; ++itr)
    {
      std::string ext = itr->path().extension().string();
      if(ext.compare(".tiff") == 0)
      {
        tiff_depth_files.push_back (itr->path ().string ());
        tiff_depth_paths.push_back (itr->path ());
      }
      else
      {
        // Found non tiff file
      }

      if(verbose)
      {
        std::cout << "Extension" << itr->path().extension() << std::endl;
        std::cout << "Filename" << itr->path().filename() << std::endl;
      }
    }
  }
  else
  {
    PCL_ERROR("Depth path is not a directory\n");
    exit(-1);
  }

  sort (tiff_depth_files.begin (), tiff_depth_files.end ());
  sort (tiff_depth_paths.begin (), tiff_depth_paths.end ());
  
  std::cout << "Nb of image file to convert : " << tiff_depth_paths.size()<< std::endl; 
  
  
  for(size_t i=0; i<tiff_depth_paths.size(); i++)
  {
    
     std:: cout << "Index : " << i <<std::endl;
     
     vtkSmartPointer<vtkTIFFReader> reader = vtkSmartPointer<vtkTIFFReader>::New ();
     
     std::string depth_filename = tiff_depth_paths[i].filename().string();
     std::string depth_time = depth_filename.substr(5,7);
     vtkSmartPointer<vtkImageData> depth_data;
     vtkSmartPointer<vtkTIFFReader> depth_reader = vtkSmartPointer<vtkTIFFReader>::New ();

     // Check if the file is correct
     std::cout << "current read file is " << depth_filename<< std::endl;
     
     int read = depth_reader->CanReadFile (tiff_depth_files[i].c_str());
     // 0 can't read the file, 1 can't prove it
     if(read == 0 || read == 1)
     {
       std::cout << "We have a broken tiff file: " << tiff_depth_files[i] << std::endl;
       continue;
      }
      // 2 can read it, 3 I'm the correct reader
      if(read == 2 || read == 3)
      {
        std::cout << "We have a good tiff file: " << tiff_depth_files[i] << std::endl; 
        depth_reader->SetFileName (tiff_depth_files[i].c_str());
        depth_reader->Update ();
        depth_data = depth_reader->GetOutput ();
        processAndSave(depth_data, depth_time, use_output_path, output_path_);
      }
    } //for depth_paths
    
  return 0;
}
#endif
