#include "registration.h"
#include "vpcl_corrs_util.h"

#include <vector>
#include <string>
#include <pcl/console/parse.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <util/vpcl_io_util.h>

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
  vpcl_io_util::load_cloud(argv[1], src_points);
  PointCloud<PointNormal>::Ptr tgt_points(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud(argv[2], tgt_points);
  
  Eigen::Matrix4f tform = Eigen::Matrix4f::Identity ();
  
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
    vpcl::correspondance::findCorrespondences (src_descriptors, tgt_descriptors, *correspondences);
    std::vector<int> indices_src, indices_tgt;
    pcl::registration::getQueryIndices (*correspondences, indices_src);
    pcl::registration::getMatchIndices (*correspondences, indices_tgt);
    
    Eigen::Matrix3d src_cov;
    Eigen::Vector4d src_centroid;

    computeMeanAndCovarianceMatrix (*src_points, src_cov, src_centroid);
    double sigma_src = sqrt(src_cov.trace());
    
    
    Eigen::Matrix3d tgt_cov;
    Eigen::Vector4d tgt_centroid;
    computeMeanAndCovarianceMatrix(*tgt_points, tgt_cov, tgt_centroid);
    double sigma_tgt = sqrt(tgt_cov.trace());

    double scale = sigma_tgt/sigma_src;
    
    cout << "Scale of source: " << sigma_src << "\nScale Target: " << sigma_tgt << "\nScale: " << scale << endl;
    
    Eigen::Matrix3f R;
    R.setIdentity();
    R = (float)scale*R;
    tform.setIdentity ();
    tform.topLeftCorner (3, 3) = R;
    
    pcl::PointCloud<PointNormal>::Ptr src_scaled(new pcl::PointCloud<PointNormal>);
    transformPointCloudWithNormals (*src_points, *src_scaled, tform);
    tform.setIdentity ();
    
    // Find the transform that roughly aligns the points
    vpcl::computeInitialAlignment<PointNormal,PointNormal,FeatureType> (src_scaled, src_descriptors, tgt_points, tgt_descriptors,
                                                                        (float)min_sample_dist, (float)max_correspondence_dist,
                                                                        nr_iters, tform);
    
    
    
    // Transform the source point to align them with the target points
    pcl::transformPointCloud (*src_points, *src_points, tform);

    
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
    
    tform = vpcl::refineAlignment<PointNormal,PointNormal> (src_points, tgt_points, tform, max_correspondence_distance,
                                                            outlier_rejection_threshold, transformation_epsilon, (float)max_iterations);
    

    
    pcl::console::print_info ("Refined alignment\n");
  }
  
  // Transform the source point to align them with the target points
  pcl::transformPointCloud (*src_points, *src_points, tform);
  
  // Save output
  std::string filename;
  bool save_output = pcl::console::parse_argument (argc, argv, "-s", filename) > 0;
  if (save_output)
  {
    // Merge the two clouds
    //(*src_points) += (*tgt_points);
    
    // Save the result
    pcl::io::savePCDFile (filename, *src_points);
    
    pcl::console::print_info ("Saved registered clouds as %s\n", filename.c_str ());
  }
  // Or visualize the result
  else
  {
    pcl::console::print_info ("Starting visualizer... Close window to exit\n");
    pcl::visualization::PCLVisualizer vis;
    
//    //at this point - demean and scale for easier visualization
//    Eigen::Matrix3d tgt_full_cov;
//    Eigen::Vector4d tgt_centroid;
//    computeMeanAndCovarianceMatrix (*tgt_points,
//                                    tgt_full_cov,
//                                    tgt_centroid);
//    
//    Eigen::Matrix3f R;
//    R.setIdentity();
//    R = (1.0/tgt_full_cov.trace())*R;
//    Eigen::Matrix4f vis_tform;
//    vis_tform.setIdentity ();
//    vis_tform.topLeftCorner (3, 3) = R;
//    
//    cout << "Sigma: " << tgt_full_cov.trace() << "\nCentroid: " << tgt_centroid << endl;
//    
//        // Transform src cloud
//    PointCloud<PointNormal>::Ptr cloud_src_demean(new PointCloud<PointNormal>);
//    demeanPointCloud (*src_points, tgt_centroid.cast<float>(), *cloud_src_demean);
//    PointCloud<PointNormal>::Ptr output_src(new PointCloud<PointNormal>);
//    transformPointCloudWithNormals (*cloud_src_demean, *output_src, vis_tform.cast<float>());
//    
//    // Transform tgt cloud
//    PointCloud<PointNormal>::Ptr cloud_tgt_demean(new PointCloud<PointNormal>);
//    demeanPointCloud (*tgt_points, tgt_centroid.cast<float>(), *cloud_tgt_demean);
//    PointCloud<PointNormal>::Ptr output_tgt(new PointCloud<PointNormal>);
//    transformPointCloudWithNormals (*cloud_tgt_demean, *output_tgt, vis_tform.cast<float>());

    
    pcl::visualization::PointCloudColorHandlerCustom<PointNormal> red (src_points, 255, 0, 0);
    vis.addPointCloud (src_points, red, "src_points");
    
    pcl::visualization::PointCloudColorHandlerCustom<PointNormal> yellow (tgt_points, 255, 255, 0);
    vis.addPointCloud (tgt_points, yellow, "tgt_points");
    
    vis.resetCamera ();
    vis.spin ();
  }
  
  return (0);
}
