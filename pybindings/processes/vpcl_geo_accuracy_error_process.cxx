//:
// \file  A process that computes verticel (LE) and circular (CE) errors.
// \brief
// \author Isabel Restrepo
// \date 9/28/12

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_parameters.h>

#include <brdb/brdb_value.h>

#include <util/vpcl_io_util.h>

#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <vgl/algo/vgl_rotation_3d.h>
#include <vnl/vnl_matrix_fixed.h>
#include <bbas_pro/bbas_1d_array_float.h>

#include <vector>
#include <algorithm>

#define DEBUG_SORT

//:global variables
namespace vpcl_geo_accuracy_error_process_globals 
{
  const unsigned n_inputs_ = 6;
  const unsigned n_outputs_ = 6;
  
  typedef std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > float4_vector;
  typedef std::vector<float4_vector> float4_vector2d;
  
  typedef std::vector< std::vector<float> > float_vector2d;
  
}


//:sets input and output types
bool vpcl_geo_accuracy_error_process_cons(bprb_func_process& pro)
{
  using namespace vpcl_geo_accuracy_error_process_globals ;
  
  vcl_vector<vcl_string> input_types_(n_inputs_);
  unsigned i = 0;
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "int";
  input_types_[i++] = "vcl_string";

  
  vcl_vector<vcl_string> output_types_(n_outputs_);
  i=0;
  output_types_[i++] = "bbas_1d_array_float_sptr"; //RMSE_X
  output_types_[i++] = "bbas_1d_array_float_sptr"; //RMSE_Y
  output_types_[i++] = "bbas_1d_array_float_sptr"; //RMSE_Z
  output_types_[i++] = "bbas_1d_array_float_sptr"; //CE_90
  output_types_[i++] = "bbas_1d_array_float_sptr"; //LE_90
  output_types_[i++] = "bbas_1d_array_float_sptr"; //radius

  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}


//:the process
bool vpcl_geo_accuracy_error_process(bprb_func_process& pro)
{
  using namespace vpcl_geo_accuracy_error_process_globals;
  using namespace pcl;
  
  pcl::console::setVerbosityLevel(console::L_DEBUG);

  
  //get inputs
  unsigned i = 0;
  // Full path ply file containing the reference fiducial points
  vcl_string fiducial_geo_ply = pro.get_input<vcl_string>(i++);  //PLY file
  // Path to root_dir/ containing subforlders trial_x 
  vcl_string root_dir = pro.get_input<vcl_string>(i++);
  // descriptor name 
  vcl_string desc_name = pro.get_input<vcl_string>(i++);
  // Basename to .ply file to estimate errors without the extension -- root_dir/trial_x/desc_name/estimate_geo_ply_basename.pcd
  vcl_string estimate_ply_basename = pro.get_input<vcl_string>(i++);  //PLY file
  // Number of trials trial_x s.t x \in [0, ntrials)
  int ntrials = pro.get_input<int>(i++);
  
  //Optional transformation -- this could be a geo transformation
  Eigen::Matrix4f GeoTform;
  GeoTform.setIdentity ();
  vcl_string Tfile = "";
  Tfile = pro.get_input<vcl_string>(i++);
  if (Tfile !="") {
    //Load the transformation
    vcl_ifstream Tfile_ss(Tfile.c_str());
    if(!Tfile_ss.is_open()){
      vcl_cerr << "Error: Could not open Transformation file: " << Tfile << "\n";
      return false;
	  }
    double scale = 0;
    vnl_quaternion<double> q;
    vnl_vector_fixed<double, 3> t;
    Tfile_ss >> scale;
    Tfile_ss >> q;
    vgl_rotation_3d<double> R(q);
    Tfile_ss >> t;

    vcl_cout << "Done reading transformation: scale = " << scale << "\nR = " << R << "\nt = " << t << vcl_endl;
    vnl_matrix_fixed<double, 3,3> rr = R.as_matrix();
    rr = scale * rr;
    t = scale * t;
    GeoTform(0,0) = rr(0,0); GeoTform(0,1) = rr(0,1); GeoTform(0,2) = rr(0,2); GeoTform(0,3) = t(0);
    GeoTform(1,0) = rr(1,0); GeoTform(1,1) = rr(1,1); GeoTform(1,2) = rr(1,2); GeoTform(1,3) = t(1);
    GeoTform(2,0) = rr(2,0); GeoTform(2,1) = rr(2,1); GeoTform(2,2) = rr(2,2); GeoTform(2,3) = t(2);
  }
  
    
  //Load the fiducial cloud (points only)
  PointCloud<PointXYZ>::Ptr fid_cloud(new PointCloud<PointXYZ>);
  vpcl_io_util::load_cloud<PointXYZ>(fiducial_geo_ply, fid_cloud);
  
  float4_vector2d errors(fid_cloud->points.size(), float4_vector(ntrials,Eigen::Vector4f(0,0,0,0)));
  
  
  //compute rmse_x, rmse_y, rmse_z, rmse_r
  float_vector2d rmse_x_all_trials(fid_cloud->points.size(), std::vector<float>(ntrials, 0.0f));
  float_vector2d rmse_y_all_trials(fid_cloud->points.size(), std::vector<float>(ntrials, 0.0f));
  float_vector2d rmse_z_all_trials(fid_cloud->points.size(), std::vector<float>(ntrials, 0.0f));
  float_vector2d rmse_r_all_trials(fid_cloud->points.size(), std::vector<float>(ntrials, 0.0f));

  
  std::vector<float> rmse_x(fid_cloud->points.size(),0.0f);
  std::vector<float> rmse_y(fid_cloud->points.size(),0.0f);
  std::vector<float> rmse_z(fid_cloud->points.size(),0.0f);
  std::vector<float> CE_90(fid_cloud->points.size(),0.0f);
  std::vector<float> LE_90(fid_cloud->points.size(),0.0f);


  
  for (unsigned trial= 0; trial < ntrials; trial++) {
    
    vcl_cout << "\n*****************Trial:" << trial << vcl_endl ;

    vcl_stringstream estimate_ply ;
    estimate_ply << root_dir << "/trial_" << trial << "/" << desc_name << "/" << estimate_ply_basename << ".pcd";
    
    //Load the estimate cloud (points only) -- ground truth normals are not available 
    PointCloud<PointXYZ>::Ptr estimate_cloud(new PointCloud<PointXYZ>);
    vpcl_io_util::load_cloud<PointXYZ>(estimate_ply.str(), estimate_cloud);
    
    // Geo register the estimated point cloid
    PointCloud<PointXYZ>::Ptr geo_estimate_cloud(new PointCloud<PointXYZ>);
    transformPointCloud (*estimate_cloud, *geo_estimate_cloud, GeoTform);
    vcl_stringstream geo_estimate_ply ;
    geo_estimate_ply << root_dir << "/trial_" << trial << "/" << desc_name << "/"  << estimate_ply_basename << "_geo.pcd";
    vpcl_io_util::save_cloud<PointXYZ>(geo_estimate_ply.str(), estimate_cloud);
   
    // Vectors to hold knn search results
    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);
    
    // For each fiducial point
    vcl_cout << "Distances: ";
    for (unsigned p_idx = 0; p_idx < fid_cloud->points.size(); p_idx++)
    {
      
      Eigen::Vector4f p_fid(fid_cloud->points[p_idx].x,
                            fid_cloud->points[p_idx].y,
                            fid_cloud->points[p_idx].z, 0);

      // Find its nearest neighbor in the target
      KdTreeFLANN<PointXYZ, flann::L2_Simple<float> > tree;
      tree.setInputCloud (geo_estimate_cloud);
      tree.nearestKSearch (fid_cloud->points[p_idx], 1, nn_indices, nn_dists);
      
      Eigen::Vector4f p_est(geo_estimate_cloud->points[nn_indices[0]].x,
                            geo_estimate_cloud->points[nn_indices[0]].y,
                            geo_estimate_cloud->points[nn_indices[0]].z, 0);
      
      // Calculate the fitness score
      Eigen::Vector4f p_delta = p_est - p_fid;
//      vcl_cout << "Fiducial Point: " << fid_cloud->points[p_idx]
//      << "\nClosest Point: " << p_est
//      vcl_cout << "Distance:" << p_delta << nn_dists[0] << vcl_endl;
      vcl_cout << " " << nn_dists[0];

      
      
      errors[p_idx][trial] = p_delta;
      //compute rmse_x, rmse_y, rmse_z, rmse_r
      rmse_x_all_trials[p_idx][trial] = p_delta[0]*p_delta[0];
      rmse_y_all_trials[p_idx][trial] = p_delta[1]*p_delta[1];

      rmse_z_all_trials[p_idx][trial] = sqrt(p_delta[2]*p_delta[2]);
      rmse_r_all_trials[p_idx][trial] = sqrt(p_delta[0]*p_delta[0] + p_delta[1]*p_delta[1]);
      
      rmse_x[p_idx] += p_delta[0]*p_delta[0];
      rmse_y[p_idx] += p_delta[1]*p_delta[1];
      rmse_z[p_idx] += p_delta[2]*p_delta[2];
    }
  }
  
  
  //Empirical CE_90 and LE_90
  int idx90 = floor(ntrials*(0.9f)) - 1; // -1 because indeces start at 0
  vcl_cout << "Float index at 90 %: " << ntrials*(0.90f) << ". Value used instead: " << idx90 << vcl_endl;
  for (unsigned p_idx = 0; p_idx < fid_cloud->points.size(); p_idx++) {
    
#ifdef DEBUG_SORT
    vector<float>::iterator it;
    vcl_cout << "Before sorting rmse_x_all_trials[" << p_idx << "]:";
    for (it=rmse_x_all_trials[p_idx].begin(); it!=rmse_x_all_trials[p_idx].end(); ++it)
      vcl_cout << " " << *it;
    vcl_cout << vcl_endl;
#endif
    sort (rmse_x_all_trials[p_idx].begin(), rmse_x_all_trials[p_idx].end());
#ifdef DEBUG_SORT
    vcl_cout << "After sorting rmse_x_all_trials[" << p_idx << "]:";
    for (it=rmse_x_all_trials[p_idx].begin(); it!=rmse_x_all_trials[p_idx].end(); ++it)
      vcl_cout << " " << *it;
    vcl_cout << vcl_endl;
#endif
#ifdef DEBUG_SORT
    vcl_cout << "Before sorting rmse_y_all_trials[" << p_idx << "]:";
    for (it=rmse_y_all_trials[p_idx].begin(); it!=rmse_y_all_trials[p_idx].end(); ++it)
      vcl_cout << " " << *it;
    vcl_cout << vcl_endl;
#endif
    sort (rmse_y_all_trials[p_idx].begin(), rmse_y_all_trials[p_idx].end());
#ifdef DEBUG_SORT
    vcl_cout << "After sorting rmse_y_all_trials[" << p_idx << "]:";
    for (it=rmse_y_all_trials[p_idx].begin(); it!=rmse_y_all_trials[p_idx].end(); ++it)
      vcl_cout << " " << *it;
    vcl_cout << vcl_endl;
#endif

    
    
    
    
#ifdef DEBUG_SORT
//    vector<float>::iterator it;
    vcl_cout << "Before sorting rmse_z_all_trials[" << p_idx << "]:";
    for (it=rmse_z_all_trials[p_idx].begin(); it!=rmse_z_all_trials[p_idx].end(); ++it)
      vcl_cout << " " << *it;
    vcl_cout << vcl_endl;
#endif
    sort (rmse_z_all_trials[p_idx].begin(), rmse_z_all_trials[p_idx].end());
#ifdef DEBUG_SORT
    vcl_cout << "After sorting rmse_z_all_trials[" << p_idx << "]:";
    for (it=rmse_z_all_trials[p_idx].begin(); it!=rmse_z_all_trials[p_idx].end(); ++it)
      vcl_cout << " " << *it;
    vcl_cout << vcl_endl;
#endif
    LE_90[p_idx] = rmse_z_all_trials[p_idx][idx90];
#ifdef DEBUG_SORT
    vcl_cout << "Before sorting rmse_r_all_trials[" << p_idx << "]:";
    for (it=rmse_r_all_trials[p_idx].begin(); it!=rmse_r_all_trials[p_idx].end(); ++it)
      vcl_cout << " " << *it;
    vcl_cout << vcl_endl;
#endif
    sort (rmse_r_all_trials[p_idx].begin(), rmse_r_all_trials[p_idx].end());
#ifdef DEBUG_SORT
    vcl_cout << "After sorting rmse_r_all_trials[" << p_idx << "]:";
    for (it=rmse_r_all_trials[p_idx].begin(); it!=rmse_r_all_trials[p_idx].end(); ++it)
      vcl_cout << " " << *it;
    vcl_cout << vcl_endl;
#endif
    CE_90[p_idx] = rmse_r_all_trials[p_idx][idx90];
    
    vcl_cout << "LE_90: " << LE_90[p_idx] << " CE_90: " << CE_90[p_idx] << vcl_endl;

  }
  
  // normalize and insert into output arrays
  bbas_1d_array_float * rmse_x_bbas = new bbas_1d_array_float(fid_cloud->points.size());
  bbas_1d_array_float * rmse_y_bbas = new bbas_1d_array_float(fid_cloud->points.size());
  bbas_1d_array_float * rmse_z_bbas = new bbas_1d_array_float(fid_cloud->points.size());
  bbas_1d_array_float * CE_90_bbas = new bbas_1d_array_float(fid_cloud->points.size());
  bbas_1d_array_float * LE_90_bbas = new bbas_1d_array_float(fid_cloud->points.size());
  bbas_1d_array_float * radius = new bbas_1d_array_float(fid_cloud->points.size());

  Eigen::Vector4f geo_origin = GeoTform*Eigen::Vector4f(0,0,0,1);   
  
  for (unsigned p_idx = 0; p_idx < fid_cloud->points.size(); p_idx++) {
    rmse_x_bbas->data_array[p_idx] = sqrt(rmse_x[p_idx]/fid_cloud->points.size());
    rmse_y_bbas->data_array[p_idx] = sqrt(rmse_y[p_idx]/fid_cloud->points.size());
    rmse_z_bbas->data_array[p_idx] = sqrt(rmse_z[p_idx]/fid_cloud->points.size());
    CE_90_bbas->data_array[p_idx]  = CE_90[p_idx];
    LE_90_bbas->data_array[p_idx]  = LE_90[p_idx];
    radius->data_array[p_idx] = (Eigen::Vector4f(fid_cloud->points[p_idx].x,fid_cloud->points[p_idx].y,fid_cloud->points[p_idx].z, 1) - geo_origin).norm();

  }
  
  
  //store output
  pro.set_output_val<bbas_1d_array_float_sptr>(0, rmse_x_bbas);
  pro.set_output_val<bbas_1d_array_float_sptr>(1, rmse_y_bbas);
  pro.set_output_val<bbas_1d_array_float_sptr>(2, rmse_z_bbas);
  pro.set_output_val<bbas_1d_array_float_sptr>(3, CE_90_bbas);
  pro.set_output_val<bbas_1d_array_float_sptr>(4, LE_90_bbas);
  pro.set_output_val<bbas_1d_array_float_sptr>(5, radius);

  
  return true;
}