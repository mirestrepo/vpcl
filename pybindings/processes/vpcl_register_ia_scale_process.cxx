//:
// \file
// \brief A process that performs initial aligment between 2 point clouds using descriptors
// \author Isabel Restrepo
// \date 8/17/12

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_parameters.h>

#include <brdb/brdb_value.h>


#include <registration/registration.h>
#include <registration/vpcl_corrs_util.h>
#include <util/vpcl_io_util.h>


#include <vector>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

#include <pcl/common/centroid.h>

//:global variables
namespace vpcl_register_ia_scale_process_globals 
{
  const unsigned n_inputs_ = 10;
  const unsigned n_outputs_ = 0;
}


//:sets input and output types
bool vpcl_register_ia_scale_process_cons(bprb_func_process& pro)
{
  using namespace vpcl_register_ia_scale_process_globals ;
  
  vcl_vector<vcl_string> input_types_(n_inputs_);
  unsigned i =0;
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "double";
  input_types_[i++] = "double";
  input_types_[i++] = "int";
  
  vcl_vector<vcl_string> output_types_(n_outputs_);
  
  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}


//:the process
bool vpcl_register_ia_scale_process(bprb_func_process& pro)
{
  using namespace vpcl_register_ia_scale_process_globals;
  
  pcl::console::setVerbosityLevel(console::L_DEBUG);
  
  //get inputs
  unsigned i=0;
  vcl_string src_fname = pro.get_input<vcl_string>(i++);
  vcl_string tgt_fname = pro.get_input<vcl_string>(i++);
  vcl_string src_features_fname = pro.get_input<vcl_string>(i++);
  vcl_string tgt_features_fname = pro.get_input<vcl_string>(i++);
  vcl_string tform_cloud_fname = pro.get_input<vcl_string>(i++);
  vcl_string tform_fname = pro.get_input<vcl_string>(i++);
  vcl_string descriptor_type = pro.get_input<vcl_string>(i++);
  double min_sample_dist = pro.get_input<double>(i++);
  double max_correspondence_dist = pro.get_input<double>(i++);
  int nr_iters = pro.get_input<int>(i++);
  
  cout <<  "\nSource: " << src_fname << "\nTarget : " << tgt_fname
  << "Source descriptors: " << src_features_fname << "\nTarget descriptors: " << tgt_features_fname
  <<"\nmin_sample_dist: " << min_sample_dist << "\nmax_correspondence_dist:" << max_correspondence_dist << "\nnr_iters: " <<nr_iters <<endl ;
  
  //Load input point clouds
  PointCloud<PointNormal>::Ptr src_points(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud<PointNormal>(src_fname, src_points);
  PointCloud<PointNormal>::Ptr tgt_points(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud<PointNormal>(tgt_fname, tgt_points);
  
  CorrespondencesPtr correspondences (new Correspondences);
  float scale = 0.0f;
  Eigen::Matrix4f tform;
  console::TicToc tt;
  double corrs_time, scale_time, trans_time = 0.0;
  
  //Load descriptos
  if (descriptor_type == "FPFH") {
    typedef FPFHSignature33 FeatureType;
    
    PointCloud<FeatureType>::Ptr src_descriptors(new PointCloud<FeatureType>);
    if (pcl::io::loadPCDFile (src_features_fname, *src_descriptors) < 0)
      return (false);
    PointCloud<FeatureType>::Ptr tgt_descriptors(new PointCloud<FeatureType>);
    if (pcl::io::loadPCDFile (tgt_features_fname, *tgt_descriptors) < 0)
      return (false);
   
    tt.tic();
    vpcl::correspondance::findCorrespondences<FeatureType>(src_descriptors, tgt_descriptors, *correspondences);
    std::vector<int> indices_src, indices_tgt;
    pcl::registration::getQueryIndices (*correspondences, indices_src);
    pcl::registration::getMatchIndices (*correspondences, indices_tgt);
    
    corrs_time =tt.toc();
    tt.tic();
    Eigen::Matrix3d src_cov;
    Eigen::Vector4d src_centroid;
    computeMeanAndCovarianceMatrix (*src_points, indices_src, src_cov, src_centroid);
    Eigen::Vector3d src_ev;
    computeRoots(src_cov, src_ev);
    double sigma_src = sqrt(src_ev[0]);
        
    Eigen::Matrix3d tgt_cov;
    Eigen::Vector4d tgt_centroid;
    computeMeanAndCovarianceMatrix(*tgt_points, indices_tgt, tgt_cov, tgt_centroid);
    Eigen::Vector3d tgt_ev;
    computeRoots(tgt_cov, tgt_ev);
    double sigma_tgt = sqrt(tgt_ev[0]);
    
    scale = sigma_tgt/sigma_src;
    scale_time = tt.toc();
    
    
    cout << "Scale of source: " << sigma_src << "\nScale Target: " << sigma_tgt << "\nScale: " << scale << endl;
    
    tform.setIdentity ();
    tform.topLeftCorner (3, 3) *= Eigen::Matrix3f::Identity() * scale;;
    
    pcl::PointCloud<PointNormal>::Ptr src_scaled(new pcl::PointCloud<PointNormal>);
    transformPointCloudWithNormals (*src_points, *src_scaled, tform);
    tform.setIdentity ();
    
   
    // Find the transform that roughly aligns the points
    tt.tic();
    vpcl::computeInitialAlignment<PointNormal,PointNormal,FeatureType> (src_scaled, src_descriptors, tgt_points, tgt_descriptors,
                                                                        (float)min_sample_dist, (float)max_correspondence_dist,
                                                                        nr_iters, tform);
    trans_time = tt.toc();
    pcl::console::print_info ("Computed initial alignment\n");
      
  }
  if (descriptor_type == "SHOT") {
    
    typedef SHOT352 FeatureType;
    
    cout << "Loading Descriptors" << endl;

    PointCloud<FeatureType>::Ptr src_descriptors(new PointCloud<FeatureType>);
    if (pcl::io::loadPCDFile (src_features_fname, *src_descriptors) < 0)
      return (false);
    PointCloud<FeatureType>::Ptr tgt_descriptors(new PointCloud<FeatureType>);
    if (pcl::io::loadPCDFile (tgt_features_fname, *tgt_descriptors) < 0)
      return (false);
    
    cout << "Done Loading Descriptors" << endl;

    tt.tic();
    
    //remove possible NAN descriptors
    std::vector<int> indeces_src;
    vpcl::correspondance::removeNaNDescriptors<PointNormal, FeatureType, 352>(*src_points, *src_descriptors, *src_points, *src_descriptors, indeces_src);
    std::vector<int> indeces_tgt;
    vpcl::correspondance::removeNaNDescriptors<PointNormal, FeatureType, 352>(*tgt_points, *tgt_descriptors, *tgt_points, *tgt_descriptors, indeces_tgt);

    cout << "Done Removing NAN" << endl;
    
    //seems too slow for SHOT
//    vpcl::correspondance::findCorrespondences<FeatureType>(src_descriptors, tgt_descriptors, *correspondences);
    
   std::vector<int> source2target;
    std::vector<int> target2source;
    vpcl::correspondance::findCorrespondences<FeatureType> (src_descriptors, tgt_descriptors, source2target, 100);
    vpcl::correspondance::findCorrespondences<FeatureType> (tgt_descriptors, src_descriptors, target2source, 100);
    vpcl::correspondance::determineReciprocalCorrespondances <FeatureType> (source2target, target2source, correspondences);
    cout << "Done Computing Correspondance" << endl;

    
    std::vector<int> indices_src, indices_tgt;
    pcl::registration::getQueryIndices (*correspondences, indices_src);
    pcl::registration::getMatchIndices (*correspondences, indices_tgt);
    
    corrs_time =tt.toc();
    tt.tic();
    Eigen::Matrix3d src_cov;
    Eigen::Vector4d src_centroid;
    computeMeanAndCovarianceMatrix (*src_points, indices_src, src_cov, src_centroid);
    Eigen::Vector3d src_ev;
    computeRoots(src_cov, src_ev);
    double sigma_src = sqrt(src_ev[0]);
    
    Eigen::Matrix3d tgt_cov;
    Eigen::Vector4d tgt_centroid;
    computeMeanAndCovarianceMatrix(*tgt_points, indices_tgt, tgt_cov, tgt_centroid);
    Eigen::Vector3d tgt_ev;
    computeRoots(tgt_cov, tgt_ev);
    double sigma_tgt = sqrt(tgt_ev[0]);
    
    scale = sigma_tgt/sigma_src;
    scale_time = tt.toc();
    
    
    cout << "Scale of source: " << sigma_src << "\nScale Target: " << sigma_tgt << "\nScale: " << scale << endl;
    
    tform.setIdentity ();
    tform.topLeftCorner (3, 3) *= Eigen::Matrix3f::Identity() * scale;;
    
    pcl::PointCloud<PointNormal>::Ptr src_scaled(new pcl::PointCloud<PointNormal>);
    transformPointCloudWithNormals (*src_points, *src_scaled, tform);
    tform.setIdentity ();
    
    
    // Find the transform that roughly aligns the points
    tt.tic();
    vpcl::computeInitialAlignment<PointNormal,PointNormal,FeatureType> (src_scaled, src_descriptors, tgt_points, tgt_descriptors,
                                                                        (float)min_sample_dist, (float)max_correspondence_dist,
                                                                        nr_iters, tform);
    trans_time = tt.toc();
    pcl::console::print_info ("Computed initial alignment\n");
    
  }
   
  // Transform the source point to align them with the target points
  tform.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * scale;
  pcl::transformPointCloud (*src_points, *src_points, tform);
    
  // Save output
  pcl::io::savePCDFile (tform_cloud_fname, *src_points); 
  pcl::console::print_info ("Saved registered clouds as %s\n", tform_cloud_fname.c_str ());
   
  ofstream ofs(tform_fname.c_str());
  ofs << scale << '\n'
  << tform << '\n';
  
  int lastindex = tform_fname.find_last_of(".");
  string corr_fname = tform_fname.substr(0, lastindex);
  string time_fname = corr_fname;
  corr_fname += "_corrs.txt";
  time_fname += "_time.txt";
  
  ofstream time_ofs;
  time_ofs.open(time_fname.c_str());
  time_ofs << "Corrs time: " << corrs_time << "\nScale time: " << scale_time << "\nTrans Time: " << trans_time;
  ofs.close();
  
  vpcl_io_util::saveCorrespondences(corr_fname, correspondences);
  
  return true;
  
}