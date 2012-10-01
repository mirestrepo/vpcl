#include "registration.h"
#include "vpcl_corrs_util.h"

#include <vector>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <util/vpcl_io_util.h>
#include <pcl/registration/transformation_estimation_svd.h>


using namespace pcl;

int
main (int argc, char ** argv)
{
  if (argc < 2)
  {
    pcl::console::print_info ("Syntax is: %s source target <options>\n", argv[0]);
    pcl::console::print_info ("  where options are:\n");
    pcl::console::print_info ("    -i min_sample_dist,max_dist,nr_iters ................ Compute initial alignment\n");
    pcl::console::print_info ("    -r max_dist,rejection_thresh,tform_eps,max_iters ............. Refine alignment\n");
    pcl::console::print_info ("    -s output.pcd ........................... Save the registered and merged clouds\n");
//    pcl::console::print_info ("Note: The inputs (source and target) must be specified without the .pcd extension\n");
    
    return (1);
  }
  
  cout << "Number of arguments: " << argc << endl;
  
  // Load the points
  PointCloud<PointNormal>::Ptr src_points(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud<PointNormal>(argv[1], src_points);
  PointCloud<PointNormal>::Ptr tgt_points(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud<PointNormal>(argv[2], tgt_points);
  
  //Globals to keep cumulative scale and transfomation
  Eigen::Matrix4f tform = Eigen::Matrix4f::Identity ();
  double scale = 1.0;
  
  // Compute the intial alignment
  float min_sample_dist, max_correspondence_dist, nr_iters;
  bool compute_intial_alignment =
  pcl::console::parse_3x_arguments (argc, argv, "-i", min_sample_dist, max_correspondence_dist, nr_iters, true) > 0;
  cout << "Compute Aligment: " << compute_intial_alignment << endl;
  if (compute_intial_alignment)
  {
    if (argc < 4)
    {
      pcl::console::print_info ("Syntax is: %s source target feature_source features_target <options>\n", argv[0]);
      pcl::console::print_info ("  where options are:\n");
      pcl::console::print_info ("    -i min_sample_dist,max_dist,nr_iters ................ Compute initial alignment\n");
      pcl::console::print_info ("    -r max_dist,rejection_thresh,tform_eps,max_iters ............. Refine alignment\n");
      pcl::console::print_info ("    -s output.pcd ........................... Save the registered and merged clouds\n");
//      pcl::console::print_info ("Note: The inputs (source and target) must be specified without the .pcd extension\n");
      
      return (1);
    }
    
    cout << "Source descriptors: " << argv[3] << "\nTarget descriptors: " << argv[4]
    <<"\nmin_sample_dist: " << min_sample_dist << "\nmax_correspondence_dist:" << max_correspondence_dist << "\nnr_iters: " <<nr_iters <<endl ;
    
    //Load local descriptors
    typedef FPFHSignature33 FeatureType;

    pcl::PointCloud<FeatureType>::Ptr src_descriptors(new PointCloud<FeatureType>);
    if (pcl::io::loadPCDFile (argv[3], *src_descriptors) < 0)
      return (false);
    PointCloud<FeatureType>::Ptr tgt_descriptors(new PointCloud<FeatureType>);
    if (pcl::io::loadPCDFile (argv[4], *tgt_descriptors) < 0)
      return (false);
    
    
    // Adjust the scale of the src to be that of the target
    CorrespondencesPtr correspondences (new Correspondences);
    vpcl::correspondance::findCorrespondences<FeatureType> (src_descriptors, tgt_descriptors, *correspondences);
    std::vector<int> indices_src, indices_tgt;
    pcl::registration::getQueryIndices (*correspondences, indices_src);
    pcl::registration::getMatchIndices (*correspondences, indices_tgt);
    
    Eigen::Matrix3d src_cov;
    Eigen::Vector4d src_centroid;
    computeMeanAndCovarianceMatrix (*src_points, indices_src, src_cov, src_centroid);
    double sigma_src = sqrt(src_cov.trace());
    
    
    Eigen::Matrix3d tgt_cov;
    Eigen::Vector4d tgt_centroid;
    computeMeanAndCovarianceMatrix(*tgt_points, indices_tgt, tgt_cov, tgt_centroid);
    double sigma_tgt = sqrt(tgt_cov.trace());

//    scale = sigma_tgt/sigma_src;
    scale = 73.0;
        
    cout << "Scale of source: " << sigma_src << "\nScale Target: " << sigma_tgt << "\nScale: " << scale << endl;
    
    Eigen::Matrix3f R;
    R.setIdentity();
    R = (float)scale*R;
    tform.setIdentity ();
    tform.topLeftCorner (3, 3) = R;
    
    pcl::PointCloud<PointNormal>::Ptr src_scaled(new pcl::PointCloud<PointNormal>);
    transformPointCloudWithNormals (*src_points, *src_scaled, tform);
    tform.setIdentity ();
    
    
    Eigen::Matrix3f src_covf;
    Eigen::Vector4f src_centroidf = src_centroid.cast<float>();
    computeCovarianceMatrix (*src_points, src_centroidf, src_covf);
    sigma_src = sqrt(src_covf.trace());
    cout << "Scale of source after normalization: " << sigma_src << endl;

    // Find the transform that roughly aligns the points
    pcl::registration::TransformationEstimationSVD<PointNormal, PointNormal> trans_est;
    trans_est.estimateRigidTransformation (*src_scaled, *tgt_points, *correspondences, tform);
    
    pcl::console::print_info ("Computed initial alignment\n");
  }
  
  // Refine the initial alignment
  std::string params_string;
  bool refine_alignment = pcl::console::parse_argument (argc, argv, "-r", params_string) > 0;
  if (refine_alignment)
  {
    std::vector<std::string> tokens;
    boost::split (tokens, params_string, boost::is_any_of (","), boost::token_compress_on);
    assert (tokens.size () == 4);
    float max_correspondence_distance = atof(tokens[0].c_str ());
    float outlier_rejection_threshold = atof(tokens[1].c_str ());
    float transformation_epsilon = atoi(tokens[2].c_str ());
    int max_iterations = atoi(tokens[3].c_str ());
    
    vpcl::refineAlignmentScale<PointNormal,PointNormal> (src_points, tgt_points, tform, max_correspondence_distance,
                                                         outlier_rejection_threshold, transformation_epsilon,0.0, (float)max_iterations,
                                                         20, 1.0, scale, tform);
       
    pcl::console::print_info ("Refined alignment\n");
  }
  
  // Transform the source point to align them with the target points
  tform.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * scale;
  pcl::transformPointCloud (*src_points, *src_points, tform);
  
  // Save output
  std::string filename;
  bool save_output = pcl::console::parse_argument (argc, argv, "-s", filename) > 0;
  if (save_output)
  {  
    // Save the result
    pcl::io::savePCDFile (filename, *src_points);
    
    pcl::console::print_info ("Saved registered clouds as %s\n", filename.c_str ());
  }
  // Or visualize the result
  else
  {
    pcl::console::print_info ("Starting visualizer... Close window to exit\n");
    pcl::visualization::PCLVisualizer vis;
       
    pcl::visualization::PointCloudColorHandlerCustom<PointNormal> red (src_points, 255, 0, 0);
    vis.addPointCloud (src_points, red, "src_points");
    
    pcl::visualization::PointCloudColorHandlerCustom<PointNormal> yellow (tgt_points, 255, 255, 0);
    vis.addPointCloud (tgt_points, yellow, "tgt_points");
    
    vis.resetCamera ();
    vis.spin ();
  }
  
  return (0);
}
