//:
// \file
// \brief  A process that runs Iterative Closest Point on two Point Clouds.
// \author Isabel Restrepo
// \date 8/17/12
// This process assumes that the clouds are close. Use vpcl_register_ia_process
// to obtain an initial aligment

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_parameters.h>

#include <brdb/brdb_value.h>

#include <registration/registration.h>
#include <registration/vpcl_corrs_util.h>
#include <util/vpcl_io_util.h>

#include <string>
#include <pcl/console/parse.h>
#include <pcl/console/print.h>
#include <pcl/console/time.h>

#include <pcl/common/centroid.h>

//:global variables
namespace vpcl_register_icp_process_globals 
{
  const unsigned n_inputs_ = 12;
  const unsigned n_outputs_ = 0;
}


//:sets input and output types
bool vpcl_register_icp_process_cons(bprb_func_process& pro)
{
  using namespace vpcl_register_icp_process_globals ;
  
  vcl_vector<vcl_string> input_types_(n_inputs_);
  unsigned i =0;
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "double";
  input_types_[i++] = "double";
  input_types_[i++] = "double";
  input_types_[i++] = "int";
  input_types_[i++] = "double";
  input_types_[i++] = "bool";
  input_types_[i++] = "bool";
  input_types_[i++] = "bool";
  
  vcl_vector<vcl_string> output_types_(n_outputs_);
  
  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}


//:the process
bool vpcl_register_icp_process(bprb_func_process& pro)
{
  using namespace vpcl_register_icp_process_globals;
  
  pcl::console::setVerbosityLevel(console::L_DEBUG);
  
  //get inputs
  unsigned i=0;
  vcl_string src_fname = pro.get_input<vcl_string>(i++);
  vcl_string tgt_fname = pro.get_input<vcl_string>(i++);
  vcl_string tform_cloud_fname = pro.get_input<vcl_string>(i++);
  vcl_string tform_fname = pro.get_input<vcl_string>(i++);
  double max_correspondence_dist = pro.get_input<double>(i++);
  double translation_epsilon = pro.get_input<double>(i++);
  double rotation_epsilon = pro.get_input<double>(i++);
  int max_iterations = pro.get_input<int>(i++);
  double max_normal_angle = pro.get_input<double>(i++);
  bool reject_normals = pro.get_input<bool>(i++);
  bool use_lm = pro.get_input<bool>(i++);
  bool compute_scale = pro.get_input<bool>(i++);


  
  cout <<  "\nSource: " << src_fname << "\nTarget : " << tgt_fname
  << "\nmax_correspondence_dist:" << max_correspondence_dist
  <<"\ntranslation_epsilon: " << translation_epsilon
  <<"\nrotation_epsilon: " << rotation_epsilon
  <<"\nmax_normal_angle: " << max_normal_angle
  <<"\nreject_normals: " << reject_normals
  <<"\nuse_lm: " << use_lm
  << "\nmax_iterations: " <<max_iterations
  << "ncompute_scale:" << compute_scale <<endl ;
  
  //Load input point clouds
  PointCloud<PointNormal>::Ptr src_points(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud<PointNormal>(src_fname, src_points);
  PointCloud<PointNormal>::Ptr tgt_points(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud<PointNormal>(tgt_fname, tgt_points);

  bool converged = false;
  double score = -1.0;
  Eigen::Matrix4d tform;
  tform.setIdentity ();
  console::TicToc tt;
  tt.tic();
  
//  tform = vpcl::refineAlignment<PointNormal,PointNormal> (src_points, tgt_points, tform, max_correspondence_dist,
//                                                          outlier_rejection_threshold, transformation_epsilon, euclidean_epsilon, max_iterations, converged, score);
  
  int iterations = vpcl::icp_with_rejection<PointNormal>(src_points, tgt_points, tform, max_iterations, translation_epsilon, rotation_epsilon, max_correspondence_dist, true,  max_normal_angle, reject_normals, use_lm, compute_scale);
  
  double trans_time = tt.toc();
  pcl::console::print_info ("Refined alignment\n");
  
  // Transform the source point to align them with the target points
  Eigen::Matrix4f tform_f = tform.cast<float>();
  pcl::transformPointCloud (*src_points, *src_points, tform_f);
  
  // Save output
  pcl::io::savePCDFile (tform_cloud_fname, *src_points);
  pcl::console::print_info ("Saved registered clouds as %s\n", tform_cloud_fname.c_str ());
  
  ofstream ofs(tform_fname.c_str());
  ofs << tform << '\n';
  
  int lastindex = tform_fname.find_last_of(".");
  string time_fname = tform_fname.substr(0, lastindex);
  time_fname += "_time.txt";
  
  ofstream time_ofs;
  time_ofs.open(time_fname.c_str());
  time_ofs << "Trans Time: " << trans_time << "\nNo. Iter: " << iterations;
  ofs.close();
  
  return true;
}