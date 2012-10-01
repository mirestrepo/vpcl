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


//:global variables
namespace vpcl_geo_accuracy_error_process_globals 
{
  const unsigned n_inputs_ = 4;
  const unsigned n_outputs_ = 4;
  
  typedef std::vector<Eigen::Vector4f,Eigen::aligned_allocator<Eigen::Vector4f> > Vector4f_v;
  typedef std::vector<Vector4f_v> Vector4f_v2D;
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
  input_types_[i++] = "int";
  
  vcl_vector<vcl_string> output_types_(n_outputs_);
  i=0;
  output_types_[i++] = "bbas_1d_array_float_sptr"; //RMSE_X
  output_types_[i++] = "bbas_1d_array_float_sptr"; //RMSE_Y
  output_types_[i++] = "bbas_1d_array_float_sptr"; //RMSE_Z
  output_types_[i++] = "bbas_1d_array_float_sptr"; //RMSE_R

  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}


//:the process
bool vpcl_geo_accuracy_error_process(bprb_func_process& pro)
{
  using namespace vpcl_geo_accuracy_error_process_globals;
  using namespace pcl;
  
  //get inputs
  unsigned i = 0;
  // Full path ply file containing the reference fiducial points
  vcl_string fiducial_geo_ply = pro.get_input<vcl_string>(i++);  //PLY file
  // Path to root_dir/ containing subforlders trial_x 
  vcl_string root_dir = pro.get_input<vcl_string>(i++);
  // Basename to .ply file to estimate errors -- root_dir/trial_x/estimate_geo_ply_filename
  vcl_string estimate_geo_ply_filename = pro.get_input<vcl_string>(i++);  //PLY file
  // Optional transformation -- this could be a geo transformation
//  Eigen::Matrix4f GeoTform;
//  GeoTform.setIdentity ();
//  vcl_string Tfile = "";
//  Tfile = pro.get_input<vcl_string>(i++);
//  if (Tfile !="") {
//    //Load the transformation
//    vcl_ifstream Tfile_ss(Tfile.c_str());
//    if(!Tfile_ss.is_open()){
//      vcl_cerr << "Error: Could not open Transformation file: " << Tfile << "\n";
//      return false;
//	  }
//    double scale = 0;
//    vnl_quaternion<double> q;
//    vnl_vector_fixed<double, 3> t;
//    Tfile_ss >> scale;
//    Tfile_ss >> q;
//    vgl_rotation_3d<double> R(q);
//    Tfile_ss >> t;
//    
//    vcl_cout << "Done reading transformation: scale = " << scale << "\nR = " << R << "\nt = " << t << vcl_endl;
//    vnl_matrix_fixed<double, 3,3> rr = R.as_matrix();
//    rr = scale * rr;
//    t = scale * t;
//    GeoTform(0,0) = rr(0,0); GeoTform(0,1) = rr(0,1); GeoTform(0,2) = rr(0,2); GeoTform(0,3) = t(0);
//    GeoTform(1,0) = rr(1,0); GeoTform(1,1) = rr(1,1); GeoTform(1,2) = rr(1,2); GeoTform(1,3) = t(1);
//    GeoTform(2,0) = rr(2,0); GeoTform(2,1) = rr(2,1); GeoTform(2,2) = rr(2,2); GeoTform(2,3) = t(2);
//  }
  
  // Number of trials trial_x s.t x \in [0, ntrials)
  int ntrials = pro.get_input<int>(i++);
    
  //Load the fiducial cloud (points only)
  PointCloud<PointXYZ>::Ptr fid_cloud(new PointCloud<PointXYZ>);
  vpcl_io_util::load_cloud<PointXYZ>(fiducial_geo_ply, fid_cloud);
  
  Vector4f_v2D errors(fid_cloud->points.size(), Vector4f_v(ntrials,Eigen::Vector4f(0,0,0,0)));
  
  
  //compute rmse_x, rmse_y, rmse_z, rmse_r
  std::vector<float> rmse_x(fid_cloud->points.size(),0.0f);
  std::vector<float> rmse_y(fid_cloud->points.size(),0.0f);
  std::vector<float> rmse_z(fid_cloud->points.size(),0.0f);
  std::vector<float> rmse_r(fid_cloud->points.size(),0.0f);

  
  for (unsigned trial= 0; trial <= ntrials; trial++) {
    
    vcl_stringstream estimate_ply ;
    estimate_ply << root_dir << "/trial_" << trial << "/" << estimate_geo_ply_filename;
    
    //Load the estimate cloud (points only) -- ground truth normals are not available 
    PointCloud<PointXYZ>::Ptr geo_estimate_cloud(new PointCloud<PointXYZ>);
    vpcl_io_util::load_cloud<PointXYZ>(estimate_ply.str(), geo_estimate_cloud);
    
    // Geo register the estimated point cloid
//    PointCloud<PointXYZ>::Ptr geo_estimate_cloud(new PointCloud<PointXYZ>);
//    transformPointCloud (*estimate_cloud, *geo_estimate_cloud, GeoTform);
    
    // Vectors to hold knn search results
    std::vector<int> nn_indices (1);
    std::vector<float> nn_dists (1);
    
    // For each fiducial point
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
      
      errors[p_idx][trial] = p_delta;
      //compute rmse_x, rmse_y, rmse_z, rmse_r
      rmse_x[p_idx] += p_delta[0];
      rmse_y[p_idx] += p_delta[1];
      rmse_z[p_idx] += p_delta[2];
      rmse_r[p_idx] += p_delta[0]*p_delta[0] + p_delta[1]*p_delta[1];
    }
  }
  
  // normalize and insert into output arrays
  bbas_1d_array_float * rmse_x_bbas = new bbas_1d_array_float(fid_cloud->points.size());
  bbas_1d_array_float * rmse_y_bbas = new bbas_1d_array_float(fid_cloud->points.size());
  bbas_1d_array_float * rmse_z_bbas = new bbas_1d_array_float(fid_cloud->points.size());
  bbas_1d_array_float * rmse_r_bbas = new bbas_1d_array_float(fid_cloud->points.size());

  for (unsigned p_idx = 0; p_idx < fid_cloud->points.size(); p_idx++) {
    rmse_r[p_idx] = rmse_r[p_idx]/fid_cloud->points.size();
    rmse_x_bbas->data_array[p_idx] = rmse_x[p_idx];
    rmse_y_bbas->data_array[p_idx] = rmse_y[p_idx];
    rmse_z_bbas->data_array[p_idx] = rmse_z[p_idx];
    rmse_r_bbas->data_array[p_idx] = rmse_r[p_idx];
  }
  
  //store output
  pro.set_output_val<bbas_1d_array_float_sptr>(0, rmse_x_bbas);
  pro.set_output_val<bbas_1d_array_float_sptr>(1, rmse_y_bbas);
  pro.set_output_val<bbas_1d_array_float_sptr>(2, rmse_z_bbas);
  pro.set_output_val<bbas_1d_array_float_sptr>(3, rmse_r_bbas);
  return true;
}