// This is vpcl_io_util.h
#ifndef vpcl_io_util_h
#define vpcl_io_util_h

//:
// \file
// \brief A class that implements the dbrec3d_part concept for composite parts (i.e non-leafs). 
// \author Isabel Restrepo mir@lems.brown.edu
// \date  25-Jan-2012.
//
// \verbatim
//  Modifications
//   <none yet>
// \endverbatim

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <vcl_iostream.h>
#include <vcl_string.h>
#include <vul/vul_file.h>

#include <rply/rply.h>   //.ply parser


namespace vpcl_io_util{
   
  //template <class PointType>
  bool load_cloud (const vcl_string &filename, pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
  
  template <typename DescriptorType>
  bool save_descriptors_as_txt(const vcl_string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud)
  {
    vcl_cout << "Saving to: " << filename <<vcl_endl;
      
    vcl_string file_type = vul_file::extension(filename);
   
    if (file_type == ".txt"){    
      vcl_ofstream  fos(filename.c_str());
      fos << cloud->size() << "\n";
      for(int i=0;i<cloud->size();i++)
      {
        for(int j=0;j<cloud->points[i].descriptor.size();j++)
				{
					fos << cloud->points[i].descriptor[j] << " ";
				}
        fos << "\n";
      }
			fos.close();
      vcl_cout << " Done saving descriptor cloud to file" << vcl_endl; 
      return true;
    } else {
      vcl_cout << "File type not supported: " << file_type << vcl_endl;
      return false;
    }
  }
  
  template <typename DescriptorType, int DIM>
  bool save_histogram_as_txt(const vcl_string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud)
  {
    vcl_cout << "Saving to: " << filename <<vcl_endl;
      
    vcl_string file_type = vul_file::extension(filename);
    
    if (file_type == ".txt"){    
      vcl_ofstream  fos(filename.c_str());
      fos << cloud->size() << "\n";
      for(int i=0;i<cloud->size();i++)
      {
        for(int j=0;j<DIM;j++)
				{
					fos << cloud->points[i].histogram[j] << " ";
				}
        fos << "\n";
      }
			fos.close();
      vcl_cout << " Done saving descriptor cloud to file" << vcl_endl; 
      return true;
    } else {
      vcl_cout << "File type not supported: " << file_type << vcl_endl;
      return false;
    }
  }
  
  //: The PLY reader of PCL is rather strict, so lets load the cloud on our own
  bool normals_pcd_from_ply(const vcl_string &filename, pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
  
  //: Call-back function for a "vertex" element
  int plyio_vertex_cb(p_ply_argument argument);
  
  //helper class to read in bb from file
  class ply_normals_reader
  {
  public:
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud;
    pcl::PointNormal p;
  };
  
  
}

#endif
