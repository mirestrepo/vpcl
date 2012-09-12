//:
// \brief
// \file
// \author Isabel Restrepo
// \date 8/11/12

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_parameters.h>

#include <brdb/brdb_value.h>

#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>

#include <util/vpcl_features_util.h>
#include <util/vpcl_io_util.h>
#include <registration/vpcl_transform_util.h>

#include <iostream>
#include <fstream>

using namespace pcl;
using namespace std;

//:global variables
namespace vpcl_feature_based_rigid_transform_process_globals 
{
  const unsigned n_inputs_ = 7;
  const unsigned n_outputs_ = 0;
}


//:sets input and output types
bool vpcl_feature_based_rigid_transform_process_cons(bprb_func_process& pro)
{
  using namespace vpcl_feature_based_rigid_transform_process_globals ;
  
  vcl_vector<vcl_string> input_types_(n_inputs_);
  unsigned i =0;
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";



  
  vcl_vector<vcl_string> output_types_(n_outputs_);

  
  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}


//:the process
bool vpcl_feature_based_rigid_transform_process(bprb_func_process& pro)
{
  using namespace vpcl_feature_based_rigid_transform_process_globals;
  
  //get inputs
  unsigned i=0;
  vcl_string src_fname = pro.get_input<vcl_string>(i++);
  vcl_string tgt_fname = pro.get_input<vcl_string>(i++);
  vcl_string src_features_fname = pro.get_input<vcl_string>(i++);
  vcl_string tgt_features_fname = pro.get_input<vcl_string>(i++);
  vcl_string tform_cloud_fname = pro.get_input<vcl_string>(i++);
  vcl_string tform_fname = pro.get_input<vcl_string>(i++);
  vcl_string descriptor_type = pro.get_input<vcl_string>(i++);
  
  //Load input point clouds
  PointCloud<PointNormal>::Ptr src(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud(src_fname, src);
  PointCloud<PointNormal>::Ptr tgt(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud(tgt_fname, tgt);
  
  CorrespondencesPtr correspondences (new Correspondences);
  float S = 0.0f;
  Eigen::Matrix<double, 3, 3> R;
  Eigen::Matrix<double, 3, 1> T;
  Eigen::Matrix<double, 4, 4> tgt_tform;
  
  //Load descriptos
  if (descriptor_type == "FPFH") {
    typedef FPFHSignature33 FeatureType;

    PointCloud<FeatureType>::Ptr src_fpfh(new PointCloud<FeatureType>);
    if (pcl::io::loadPCDFile (src_features_fname, *src_fpfh) < 0)
      return (false);
    PointCloud<FeatureType>::Ptr tgt_fpfh(new PointCloud<FeatureType>);
    if (pcl::io::loadPCDFile (tgt_features_fname, *tgt_fpfh) < 0)
      return (false);
    //Compute Transformation
    vpcl::transform::computeTransformation (src,
                                            tgt,
                                            src_fpfh,
                                            tgt_fpfh,
                                            S, R, T,
                                            tgt_tform,
                                            correspondences);
    
  }
  
  // Transform the data
  PointCloud<PointNormal>::Ptr output(new PointCloud<PointNormal>);
  transformPointCloudWithNormals (*src, *output, tgt_tform.cast<float>());

  //Save the output point cloud
  if (pcl::io::savePCDFile(tform_cloud_fname, *output) < 0)
    return (false);

  
  ofstream ofs(tform_fname.c_str());
  ofs << S << '\n'
  << R << '\n'
  << T << '\n'
  << tgt_tform << '\n';
  
  
  int lastindex = tform_fname.find_last_of(".");
  string corr_fname = tform_fname.substr(0, lastindex);
  corr_fname += "_corrs.txt";
  
  vpcl_io_util::saveCorrespondences(corr_fname, correspondences);

  
  return true;
}