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
  template <class PointType>
  bool  load_cloud (const string &filename, typename pcl::PointCloud<PointType>::Ptr cloud);
  
  //: Save descriptor as txt
  template <typename DescriptorType>
  bool save_descriptors_as_txt(const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud);

  
  //: For PCL version < 1.7 - Some histogram based descriptors did not have a native PCD writer
  template <typename DescriptorType, int DIM>
  bool save_histogram_as_txt(const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud);

  
  //: Save cloud. If extesnsion is .pcd, PCL writer is used. For .txt we use our custom writer
  //  This is the version used for decriptors that save a histogram inside
  template <typename DescriptorType, int DIM >
  bool save_cloud (const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud, bool binary_mode = false );
  
  //: Save cloud. If extesnsion is .pcd, PCL writer is used. For .txt we use our custom writer
  template <typename DescriptorType >
  bool save_cloud (const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud, bool binary_mode = false );
  
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
  template <class PointType>
  bool pcd_from_ply(const string &filename, typename pcl::PointCloud<PointType>::Ptr cloud);
  
  //: Call-back function for a "vertex" element
  template <class PointType>
  int plyio_vertex_cb(p_ply_argument argument);
  
  //helper class to read in bb from file
  template <class PointType>
  class ply_normals_reader
  {
  public:
    typename pcl::PointCloud<PointType>::Ptr cloud;
    PointType p;
  };
  
}

//Add the implementations
#include "vpcl_io_util.txx"

#endif
