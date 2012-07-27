#ifndef dbrec3d_pcl_features_util_h
#define dbrec3d_pcl_features_util_h

//:
// \file
// \brief 
// \author Isabel Restrepo mir@lems.brown.edu
// \date  11/3/11
//
// \verbatim
//  Modifications
//   <none yet>
// \endverbatim

#include <pcl/point_types.h>

#include <pcl/features/fpfh.h>
#include <pcl/features/spin_image.h>


#include <vcl_vector.h>
#include <vcl_iostream.h>

#include <vul/vul_timer.h>

#include <vnl/vnl_vector_fixed.h>


//: Compute Fast Point Feature Histogram Features 
void vpcl_compute_fpfh_features(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, 
                                       pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_out,
                                       double cell_length, int region_unit_radius);

//: Compute Fast Point Feature Histogram Features 
void vpcl_compute_fpfh_features(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_in, 
                                       pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud_out,
                                       double radius);

//: Put the data inside the point cloud into a stl vector.
//  This only cares about the data and not the 3-d location
void vpcl_pcd_to_vnl_vector(pcl::PointCloud<pcl::FPFHSignature33>::Ptr cloud,
                               vcl_vector<vnl_vector_fixed<double, 33> > &features);

//: Compute Spin Images
template <int DIM>
void vpcl_compute_spin_images(pcl::PointCloud<pcl::PointNormal>::Ptr xyznormals, 
                                     typename pcl::PointCloud<pcl::Histogram<DIM> >::Ptr descriptors,
                                     double radius, int image_width, double support_angle, double min_neigh, bool radial, bool angular)
{
  using namespace pcl;
  typedef Histogram<DIM> SpinImage;
  std::cout << "my image_width: " << image_width <<std::endl;
  SpinImageEstimation<PointNormal, PointNormal, SpinImage> spin_est (image_width, support_angle, min_neigh); 
  spin_est.setInputCloud(xyznormals);
  spin_est.setInputNormals(xyznormals);
  spin_est.setSearchMethod (search::KdTree<PointNormal>::Ptr (new search::KdTree<PointNormal>));
  spin_est.setRadiusSearch (radius);
  
  if (radial)
    spin_est.setRadialStructure();
  
  if (angular)
    spin_est.setAngularDomain();
  
  vul_timer timer;
  timer.mark();
  vcl_cout << "Computing Spin Image: " << vcl_endl;
  spin_est.compute (*descriptors);
  vcl_cout << "Done \n";
  timer.print(vcl_cout);  
}

#endif
