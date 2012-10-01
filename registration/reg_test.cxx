//:
// \file
// \brief A simple registration test
// \author Isabel Restrepo
// \date July 30, 2012
#include <iostream>

#include "vpcl_transform_util.h"
#include <util/vpcl_io_util.h>
#include <vul/vul_file.h>
#include <vul/vul_timer.h>

#include <pcl/features/fpfh_omp.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef PointNormal PointT;
typedef PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

CloudPtr src, tgt;


using namespace pcl;
using namespace std;
using namespace pcl::visualization;

typedef FPFHSignature33 FeatureType;
typedef PointNormal PointType;
typedef PointNormal NormalType;

boost::shared_ptr<PCLVisualizer> vis;
bool visualize = true;

void estimate_FPFH(PointCloud<PointNormal>::Ptr cloud_in,
                   PointCloud<FeatureType>::Ptr descriptors)
{
  
  float resolution=0.0022;
  float radius=30*resolution;
  
  //Estimate descriptor
  typedef FPFHSignature33 FeatureType;
  typedef PointNormal PointType;
  typedef PointNormal NormalType;
  FPFHEstimationOMP<PointType, NormalType, FeatureType>::Ptr extractor(new FPFHEstimationOMP<PointType, NormalType, FeatureType>(8));
  
  extractor->setInputCloud(cloud_in);
  extractor->setInputNormals(cloud_in);
  extractor->setSearchMethod(search::KdTree<PointType>::Ptr (new search::KdTree<PointType>));
  extractor->setRadiusSearch(radius);
  vul_timer timer;
  timer.mark();
  vcl_cout << "Descriptor extraction FPFH..." << vcl_endl;
  extractor->compute (*descriptors);
  vcl_cout << "Done \n";

}
 

void
view (const CloudConstPtr &src, const CloudConstPtr &tgt, const CorrespondencesPtr &correspondences)
{
  if (!visualize || !vis) return;
  PointCloudColorHandlerCustom<PointT> green (tgt, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> blue (src, 0, 0, 255);

  if (!vis->updatePointCloud<PointT> (src, blue, "source"))
  {
    vis->addPointCloud<PointT> (src, blue, "source");
    vis->resetCameraViewpoint ("source");
  }
  if (!vis->updatePointCloud<PointT> (tgt, green, "target")) vis->addPointCloud<PointT> (tgt, green, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.5, "source");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.7, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 6, "source");
//  TicToc tt;
////  tt.tic ();
//  if (!vis->updateCorrespondences<PointT> (src, tgt, *correspondences, 100))
//    vis->addCorrespondences<PointT> (src, tgt, *correspondences, 100, "correspondences");
//  tt.toc_print ();
  vis->setShapeRenderingProperties (PCL_VISUALIZER_LINE_WIDTH, 0.3, "correspondences");
  //vis->setShapeRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "correspondences");
  vis->spin ();
}



int main(int argc, char *argv[])
{
  
  string src_file = "/data/helicopter_providence_3_12/site_12/gauss_233_normals_t90_scili.ply";
 
  float resolution=0.0022;
  float max_dist=5.0*resolution;
  
  //create and read from file the input point cloud
  PointCloud<PointNormal>::Ptr src(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud<PointNormal>(src_file, src);
//  PointCloud<FeatureType>::Ptr src_fpfh(new PointCloud<FeatureType>);
//  estimate_FPFH(src, src_fpfh);
  
  //transform the input point cloud
  // Initialize the transformation matrix
  Eigen::Matrix4f tform;
  tform.setIdentity ();
  const double pi = std::acos(-1.0);
  tform.topLeftCorner (3, 3) = Eigen::Matrix3f (Eigen::AngleAxisf (pi, Eigen::Vector3f::UnitY ()));
  PointCloud<PointNormal>::Ptr tgt(new PointCloud<PointNormal>);
  transformPointCloudWithNormals(*src, *tgt, tform);
//  PointCloud<FeatureType>::Ptr tgt_fpfh(new PointCloud<FeatureType>);
//  estimate_FPFH(tgt, tgt_fpfh);
  
//  //Compute Transformation
//  float S = 0.0f;
//  Eigen::Matrix<double, 3, 3> R;
//  Eigen::Matrix<double, 3, 1> T;
//   Eigen::Matrix<double, 4, 4> tgt_tform;
   CorrespondencesPtr correspondences (new Correspondences);
//
//  vpcl::transform::computeTransformation (src,
//                                          tgt,
//                                          src_fpfh,
//                                          tgt_fpfh,
//                                          S,R,T,
//                                          tgt_tform,
//                                          correspondences);
//  
//  std::cerr << "Original Transform: " << "\n" << tform << std::endl;
//  std::cerr << "Rcovered Transform: " << "\n" << tgt_tform << std::endl;
//  
//  // Transform the data
//  PointCloud<PointNormal>::Ptr output(new PointCloud<PointNormal>);
//  transformPointCloudWithNormals (*src, *output, tgt_tform.cast<float>());
//
  
  // Visualize the results
  if (visualize)
    vis.reset (new PCLVisualizer ("Registration example"));
  view (tgt,src, correspondences);
  
  return 0;
  
}