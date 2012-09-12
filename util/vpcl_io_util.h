// This is vpcl_io_util.h
#ifndef vpcl_io_util_h
#define vpcl_io_util_h

//:
// \file
// \brief IO utilities for point clouds extracted from the PVM. 
// \author Isabel Restrepo mir@lems.brown.edu
// \date  25-Jan-2012.
//
// \verbatim
//  Modifications
//   <none yet>
// \endverbatim

#pragma warning(disable: 4201)


#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <iostream.h>
#include <string.h>

#include <vul/vul_file.h>

#include <rply/rply.h>   //.ply parser

using namespace std;


namespace vpcl_io_util{
   
  //: Load cloud. If extesnsion is .pcd, PCL reader is used. For .ply we use our custom reader
  bool load_cloud (const string &filename, pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
  
  
  //: Save descriptor as txt
  template <typename DescriptorType>
  bool save_descriptors_as_txt(const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud)
  {
#ifdef PCL_1_6
    cout << "Saving to: " << filename <<endl;
      
    string file_type = vul_file::extension(filename);
   
    if (file_type == ".txt"){    
      ofstream  fos(filename.c_str());
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
      cout << " Done saving descriptor cloud to file" << endl; 
      return true;
    } else {
      cout << "File type not supported: " << file_type << endl;
      return false;
    }
#else
    cout << "save_descriptors_as_txt has not been implemented for this version of PCL " <<endl;
#endif
  }
  
  //: For PCL version < 1.7 - Some histogram based descriptors did not have a native PCD writer
  template <typename DescriptorType, int DIM>
  bool save_histogram_as_txt(const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud)
  {
    cout << "Saving to: " << filename <<endl;
      
    string file_type = vul_file::extension(filename);
    
    if (file_type == ".txt"){    
      ofstream  fos(filename.c_str());
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
      cout << " Done saving descriptor cloud to file" << endl; 
      return true;
    } else {
      cout << "File type not supported: " << file_type << endl;
      return false;
    }
  }
  
  //: Save cloud. If extesnsion is .pcd, PCL writer is used. For .txt we use our custom writer
  //  This is the version used for decriptors that save a histogram inside
  template <typename DescriptorType, int DIM >
  bool save_cloud (const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud, bool binary_mode = false )
  {
    cout << "Saving: " << filename <<endl;
    
    string file_type = vul_file::extension(filename);
    if (file_type == ".pcd") {
      if (pcl::io::savePCDFile (filename, *cloud, binary_mode) < 0)
        return (false);
    }
    else if (file_type == ".txt"){
      if(!save_histogram_as_txt<DescriptorType, DIM>(filename, cloud))
        return false;
    }
    else {
      cout << "File type not supported: " << file_type << endl;
      return false;
    }
  }
  
  //: Save cloud. If extesnsion is .pcd, PCL writer is used. For .txt we use our custom writer
  template <typename DescriptorType >
  bool save_cloud (const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud, bool binary_mode = false )
  {
    cout << "Saving: " << filename <<endl;
    
    string file_type = vul_file::extension(filename);
    if (file_type == ".pcd") {
      if (pcl::io::savePCDFile (filename, *cloud, binary_mode) < 0)
        return (false);
    }
    else if (file_type == ".txt"){
      if(!save_descriptors_as_txt<DescriptorType>(filename, cloud))
        return false;
    }
    else {
      cout << "File type not supported: " << file_type << endl;
      return false;
    }
  }
  
  template<class T>
  T fromString(const std::string& s)
  {
    try {
      istringstream stream (s);
      T t;
      stream >> t;
      return t;
    }
    catch(int e){
      cout<<"Cannot convert string "<<s<<" to numeric value"<<endl;
      return 0;
    }
  }
  
  //String split helper methods....
  vector<string>& split(const string &s, char delim, vector<string> &elems);
  vector<string> split(const string &s, char delim = ' ');
  

  
  //: Load correspondances
  pcl::CorrespondencesPtr loadCorrespondences(const string& name);
  
  //: Save correspondances
  void saveCorrespondences(const string& name, pcl::CorrespondencesPtr); 
  
  //: The PLY reader of PCL is rather strict, so lets load the cloud on our own
  bool normals_pcd_from_ply(const string &filename, pcl::PointCloud<pcl::PointNormal>::Ptr cloud);
  
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
