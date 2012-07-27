//:
// \file
// \author Isabel Restrepo
// \date 11/3/11

#include "vpcl_features_util.h"

#include <pcl/search/kdtree.h> 


using namespace pcl;

void vpcl_compute_fpfh_features(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, 
                                       pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_out,
                                       double cell_length, int region_unit_radius)
{
  
  //Estimation
  FPFHEstimation<PointNormal, PointNormal, FPFHSignature33> ne;
  ne.setInputCloud (cloud_in);
  ne.setInputNormals (cloud_in);
  ne.setSearchMethod (search::KdTree<PointNormal>::Ptr (new search::KdTree<PointNormal>));
  ne.setKSearch (100);
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  ne.setRadiusSearch (cell_length*(double)(region_unit_radius));
  
  // Compute the features
  vul_timer timer;
  timer.mark();
  vcl_cout << "Computing FPFH: " << vcl_endl;
  ne.compute (*cloud_out); 
  vcl_cout << "Done \n";
  timer.print(vcl_cout);

}

void vpcl_compute_fpfh_features(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, 
                                       pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_out,
                                       double radius)
{
  //Estimation
  FPFHEstimation<PointNormal, PointNormal, FPFHSignature33> ne;
  ne.setInputCloud (cloud_in);
  ne.setInputNormals (cloud_in);
  ne.setSearchMethod (search::KdTree<PointNormal>::Ptr (new search::KdTree<PointNormal>));
  ne.setKSearch (100);
  // IMPORTANT: the radius used here has to be larger than the radius used to estimate the surface normals!!!
  ne.setRadiusSearch (0);

  // Compute the features
  vul_timer timer;
  timer.mark();
  vcl_cout << "Computing FPFH: " << vcl_endl;
  ne.compute (*cloud_out); 
  vcl_cout << "Done \n";
  timer.print(vcl_cout);
}

//: Put the data inside the point cloud into a stl vector.
//  This only cares about the data and not the 3-d location
void vpcl_pcd_to_vnl_vector(pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud,
                            vcl_vector<vnl_vector_fixed<double, 33 > > &features)
{
  for (size_t i = 0; i < cloud->points.size (); ++i) {
    vnl_vector_fixed<double, 33> hist(0.0);
    for (unsigned bin = 0; bin<33; bin++) {
      hist[bin]=(double)cloud->points[i].histogram[bin];
    }
    features.push_back(hist);
  }
}

