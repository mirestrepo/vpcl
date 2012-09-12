//:
// \file Whiten two clouds based on their correspondance
// \brief
// \author Isabel Restrepo
// \date 8/15/12

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_parameters.h>

#include <brdb/brdb_value.h>

#if 0
//:global variables
namespace vpcl_corrs_based_whitening_process_globals 
{
  const unsigned n_inputs_ = ;
  const unsigned n_outputs_ =;
}


//:sets input and output types
bool vpcl_corrs_based_whitening_process_cons(bprb_func_process& pro)
{
  using namespace vpcl_corrs_based_whitening_process_globals ;
  
  vcl_vector<vcl_string> input_types_(n_inputs_);
  input_types_[0] = ;
  
  vcl_vector<vcl_string> output_types_(n_outputs_);
  output_types_[0] = ;
  
  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}


//:the process
bool vpcl_corrs_based_whitening_process(bprb_func_process& pro)
{
  using namespace vpcl_corrs_based_whitening_process_globals;
  
  //get inputs
  unsigned i=0;
  vcl_string src_fname = pro.get_input<vcl_string>(i++);
  vcl_string tgt_fname = pro.get_input<vcl_string>(i++);
  vcl_string src_features_fname = pro.get_input<vcl_string>(i++);
  vcl_string tgt_features_fname = pro.get_input<vcl_string>(i++);
  vcl_string white_src_fname = pro.get_input<vcl_string>(i++);
  vcl_string white_tgt_fname = pro.get_input<vcl_string>(i++);
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

    //compute the correspondances
    
    
    //get the indeces from the correspondances
    std::vector<int> indices_src, indices_tgt;
    pcl::registration::getQueryIndices (*correspondences, indices_src);
    pcl::registration::getMatchIndices (*correspondences, indices_tgt);
    
    //Whiten the clouds
    Eigen::Vector4d centroid_src;
    Eigen::Vector4d centroid_tgt;
    Eigen::Matrix3d cov_src;
    Eigen::Matrix3d cov_tgt;
    
    computeMeanAndCovarianceMatrix(*src, indices_src, cov_src, centroid_src);
    computeMeanAndCovarianceMatrix(*tgt, indeces_tgt, cov_tgt, centroid_tgt);
    
    demeanPointCloud(*src, *src, centroid_src);
    demeanPointCloud(*tgt, *tgt, centroid_tgt);
    
    
    
    
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

  
  
  //store output
  pro.set_output_val<boxm_scene_base_sptr>(0, output);
  
  return true;
}

#endif