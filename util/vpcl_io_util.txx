#ifndef vpcl_io_util_txx
#define vpcl_io_util_txx

//:
// \file
// \author Isabel Restrepo
// \date  9/19/12

#include "vpcl_io_util.h"

//: Load cloud. If extesnsion is .pcd, PCL reader is used. For .ply we use our custom reader
template <class PointType>
bool vpcl_io_util::load_cloud (const string &filename, typename pcl::PointCloud<PointType>::Ptr cloud)
{
  cout << "Loading: " << filename <<endl;
  
  string file_type = vul_file::extension(filename);
  if (file_type == ".pcd") {
    if (pcl::io::loadPCDFile (filename, *cloud) < 0)
      return (false);
  }
  else if (file_type == ".ply"){
    pcl::PLYReader reader;
    cout << " Reading..." << endl;
    pcd_from_ply<PointType>(filename, cloud);
  }
  else {
    cout << "File type not supported: " << file_type << endl;
    return false;
  }
  
  
  cout << " Done:" <<  cloud->width * cloud->height << " point" << endl;
  cout << "Available dimensions: " << getFieldsList(*cloud).c_str () << endl;
  cout << "First point: " << cloud->points[0] << endl;
  
  return true;
}

//: Save descriptor as txt
template <typename DescriptorType, int DIM>
bool vpcl_io_util::save_descriptors_as_txt(const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud)
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
}

//: For PCL version < 1.7 - Some histogram based descriptors did not have a native PCD writer
template <typename DescriptorType, int DIM>
bool vpcl_io_util::save_histogram_as_txt (const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud)
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
bool vpcl_io_util::save_cloud (const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud, bool binary_mode = false )
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

//: Save cloud. If extension is .pcd, PCL writer is used. For .txt we use our custom writer
template <typename DescriptorType >
bool vpcl_io_util::save_cloud (const string &filename, typename pcl::PointCloud<DescriptorType>::Ptr cloud, bool binary_mode = false )
{
  cout << "Saving: " << filename <<endl;
  
  string file_type = vul_file::extension(filename);
  if (file_type == ".pcd") {
    if (pcl::io::savePCDFile (filename, *cloud, binary_mode) < 0){
      cerr << "Failed to save: " << filename <<endl;
      return (false);
    }
  }
  else {
    cout << "File type not supported: " << file_type << endl;
    return false;
  }
}

#endif
