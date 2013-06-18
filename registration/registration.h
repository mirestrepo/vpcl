#ifndef REGISTRATION_H
#define REGISTRATION_H


#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/default_convergence_criteria.h>
#include <pcl/registration/transformation_estimation_lm.h>
#include <pcl/registration/transformation_estimation_svd_scale.h>

#include <pcl/visualization/pcl_visualizer.h>


#include "vpcl_transform_util.h"
#include "vpcl_corrs_util.h"
#include "vpcl_transformation_estimation_svd_srt.h"

using namespace pcl::registration;
using namespace pcl::visualization;
using namespace pcl::console;



namespace vpcl {

    /* Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by finding
     * correspondences between two sets of local features. This function uses SVD to solve for the transfomation
     * Inputs:
     *   source_points
     *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
     *   source_descriptors
     *     The local descriptors for each source point
     *   target_points
     *     The "target" points, i.e., the points to which the source point cloud will be aligned
     *   target_descriptors
     *     The local descriptors for each target point
     *   min_sample_distance
     *     The minimum distance between any two random samples
     *   max_correspondence_distance
     *     The
     *   nr_interations
     *     The number of RANSAC iterations to perform
     * Return: A transformation matrix that will roughly align the points in source to the points in target
     */
  
    template <typename PointSource, typename PointTarget,  typename FeatureT>
    double 
    computeInitialAlignment (const typename pcl::PointCloud<PointSource>::Ptr & source_points, const typename pcl::PointCloud<FeatureT>::Ptr & source_descriptors,
                             const typename pcl::PointCloud<PointTarget>::Ptr & target_points, const typename pcl::PointCloud<FeatureT>::Ptr & target_descriptors,
                             float min_sample_distance, float max_correspondence_distance, int nr_iterations, int nsamples, Eigen::Matrix4f &Tresult)
    {
      pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT> sac_ia;
      sac_ia.setMinSampleDistance (min_sample_distance);
      sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
      sac_ia.setMaximumIterations (nr_iterations);
      
      sac_ia.setInputCloud (source_points);
      sac_ia.setSourceFeatures (source_descriptors);
      
      sac_ia.setInputTarget (target_points);
      sac_ia.setTargetFeatures (target_descriptors);
      
      typename boost::shared_ptr<pcl::registration::TransformationEstimationSVD<PointNormal, PointNormal, float> > te (new typename pcl::registration::TransformationEstimationSVD<PointNormal, PointNormal, float>);
      
      sac_ia.setTransformationEstimation(te);
      sac_ia.setNumberOfSamples(nsamples);

      pcl::PointCloud<PointSource> registration_output;
      sac_ia.align (registration_output);
      
      Tresult = sac_ia.getFinalTransformation ();
      return sac_ia.getFitnessScore(); 
    }
  
  /* Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by finding
   * correspondences between two sets of local features. This function uses SVD to solve for the transfomation
   * Inputs:
   *   source_points
   *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
   *   source_descriptors
   *     The local descriptors for each source point
   *   target_points
   *     The "target" points, i.e., the points to which the source point cloud will be aligned
   *   target_descriptors
   *     The local descriptors for each target point
   *   min_sample_distance
   *     The minimum distance between any two random samples
   *   max_correspondence_distance
   *     The
   *   nr_interations
   *     The number of RANSAC iterations to perform
   * Return: A transformation matrix that will roughly align the points in source to the points in target
   */
  
  template <typename PointSource, typename PointTarget,  typename FeatureT>
  double
  computeInitialAlignmentScale (const typename pcl::PointCloud<PointSource>::Ptr & source_points,
                                const typename pcl::PointCloud<FeatureT>::Ptr & source_descriptors,
                                const typename pcl::PointCloud<PointTarget>::Ptr & target_points,
                                const typename pcl::PointCloud<FeatureT>::Ptr & target_descriptors,
                                float min_sample_distance, float max_correspondence_distance,
                                int nr_iterations, int nsamples,
                                Eigen::Matrix4f &Tresult,
                                float &ransac_scale, float &avg_scale,
                                bool bound_scale=false, float min_scale=0.0f, float max_scale=0.0f)
  {
    pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT> sac_ia;
    sac_ia.setMinSampleDistance (min_sample_distance);
    sac_ia.setMaxCorrespondenceDistance (max_correspondence_distance);
    sac_ia.setMaximumIterations (nr_iterations);
    
    sac_ia.setInputCloud (source_points);
    sac_ia.setSourceFeatures (source_descriptors);
    
    sac_ia.setInputTarget (target_points);
    sac_ia.setTargetFeatures (target_descriptors);
    
    cout << "SAC_IA using SDVScale" <<endl;
    typename boost::shared_ptr<vpcl::registration::TransformationEstimationSVD_SRT<PointSource, PointTarget, float> > te (new typename vpcl::registration::TransformationEstimationSVD_SRT<PointSource, PointTarget, float>);
    
    if(bound_scale)
      te->set_scale_bounds(min_scale, max_scale);
    
    sac_ia.setTransformationEstimation(te);
    sac_ia.setNumberOfSamples(nsamples);
    

//    typedef typename pcl::SampleConsensusInitialAlignment<PointSource, PointTarget, FeatureT>::HuberPenalty Error;
//    typename boost::shared_ptr<Error> ef(new Error(max_correspondence_distance));
//    sac_ia.setErrorFunction(ef);
    
    
    pcl::PointCloud<PointSource> registration_output;
    sac_ia.align (registration_output);
    
    Tresult = sac_ia.getFinalTransformation ();
    
    double total_iter = nr_iterations + te->count_scale_out_of_bounds();
    avg_scale = (float)te->avg_scale();
    ransac_scale = (float)Tresult.topLeftCorner(3,3).col(0).norm(); //assumes isotropic scale
    
//    return sac_ia.getFitnessScore();
    return total_iter;
  }
  
  
  /* Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by finding
   * correspondences between two sets of local features
   * Inputs:
   *   source_points
   *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
   *   source_descriptors
   *     The local descriptors for each source point
   *   target_points
   *     The "target" points, i.e., the points to which the source point cloud will be aligned
   *   target_descriptors
   *     The local descriptors for each target point
   *   min_sample_distance
   *     The minimum distance between any two random samples
   *   max_correspondence_distance
   *     The
   *   nr_interations
   *     The number of RANSAC iterations to perform
   *   scale
   *     The scale of the transformation - as an input it should contain an initial estimate. 
   *     As an output, it is set to be the best scale foound
   * Return: A transformation matrix that will roughly align the points in source to the points in target
   */
  
  template <typename PointSource, typename PointTarget,  typename FeatureT>
  double
  computeInitialAlignmentScaleBruteSearch (const typename pcl::PointCloud<PointSource>::Ptr & source_points,
                                           const typename pcl::PointCloud<FeatureT>::Ptr & source_descriptors,
                                           const typename pcl::PointCloud<PointTarget>::Ptr & target_points,
                                           const typename pcl::PointCloud<FeatureT>::Ptr & target_descriptors,
                                           float min_sample_distance, float max_correspondence_distance,
                                           int sac_niter, int scale_niter, double iteration_scale_step,
                                           double &scale,  Eigen::Matrix4f &Tresult)
  {
    
    
    double init_scale = scale;
    Eigen::Matrix4f best_T; double best_scale = scale; double best_score = std::numeric_limits<double>::max ();
    
    for(int i=-(scale_niter/2);i<=(scale_niter/2);i++)
    {
      double  trial_scale= (init_scale + (double)i*(iteration_scale_step));
      cout << "Procesing Scale: " << trial_scale << endl;
      Eigen::Matrix4f Tscale = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
      Tscale.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * trial_scale;
      
      pcl::PointCloud<pcl::PointNormal>::Ptr source_scaled(new pcl::PointCloud<pcl::PointNormal>);
      pcl::transformPointCloud<pcl::PointNormal>(*source_points, *source_scaled, Tscale);
      
      double fitness_score = computeInitialAlignment<PointSource,PointTarget,FeatureT> (source_scaled, source_descriptors,
                                                                                        target_points, target_descriptors,
                                                                                        (float)min_sample_distance, (float)max_correspondence_distance,
                                                                                        sac_niter,Tresult);
      cout << "Initial Aligment transform:"<<endl<<Tresult<<endl;
      cout << "Fitness Score: "<<fitness_score<<endl;
      cout << "------------------------------------------------------------------------" << endl;
      
      if(fitness_score < best_score) {
        best_score = fitness_score;
        best_T = Tresult;
        best_scale = trial_scale;
      }
    }
    
    Tresult = best_T;
    scale = best_scale;
    
    cout << "Initial Aligment Best Transform:"<<endl<<Tresult<<endl;
    cout << "Best Fitness Score: "<<best_score<<endl;
    cout << "Best Scale: " << scale <<endl;
    cout << "------------------------------------------------------------------------" << endl;
    
    
    return best_score;
  }
  
  
  template <typename PointSource, typename PointTarget>
  int RANSACRegister(const typename pcl::PointCloud<PointSource>::Ptr& src,
                     const typename pcl::PointCloud<PointTarget>::Ptr& tgt,
                     float max_correspondence_distance,
                     Eigen::Matrix4f& Tresult)
  {
    typename pcl::SampleConsensusModelRegistration<PointSource>::Ptr sac_model(new typename pcl::SampleConsensusModelRegistration<PointSource>(src));
    sac_model->setInputTarget(tgt);
    
    pcl::RandomSampleConsensus<PointSource> ransac(sac_model);
    ransac.setDistanceThreshold(max_correspondence_distance);
    
    //upping the verbosity level to see some info
//    pcl::console::VERBOSITY_LEVEL vblvl = pcl::console::getVerbosityLevel();
//    pcl::console::setVerbosityLevel(2);
    ransac.computeModel(1);
//    pcl::console::setVerbosityLevel(0);
    
    Eigen::VectorXf coeffs;
    ransac.getModelCoefficients(coeffs);
    assert(coeffs.size() == 16);
    Tresult = Eigen::Map<Eigen::Matrix4f>(coeffs.data(),4,4);
    
    vector<int> inliers; ransac.getInliers(inliers);
    return inliers.size();
  }
  
  template <typename PointSource, typename PointTarget>
  void RANSACRegisterScale(const typename pcl::PointCloud<PointSource>::Ptr& src,
                           const typename pcl::PointCloud<PointTarget>::Ptr& tgt,
                           Eigen::Matrix4f& Tresult,
                           double& scale,
                           int num_iterations,
                           double iteration_scale_step,
                           float max_correspondence_distance)
  {
    double init_scale = scale;
    int max_inliers = 0; Eigen::Matrix4f max_T; double best_s = init_scale;
    
    for(int i=-(num_iterations/2);i<=(num_iterations/2);i++)
      //int i=0;
    {
      double trial_scale = (init_scale + (double)i*(iteration_scale_step));
      cout << "Applying scale: " << trial_scale << endl;
      Eigen::Matrix4f Tscale = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
      Tscale.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * trial_scale;
      
      typename pcl::PointCloud<PointSource>::Ptr src_scaled( new pcl::PointCloud<PointSource> );
      pcl::transformPointCloud<PointSource>(*src, *src_scaled, Tscale);
      
      int inliers_num = RANSACRegister<PointSource, PointTarget>(src_scaled,tgt,max_correspondence_distance, Tresult);
      cout << "RANSAC rigid transform:"<<endl<<Tresult.transpose()<<endl;
      cout << "RANSAC inliers:"<<inliers_num<<endl;
      cout << "------------------------------------------------------------------------" << endl;
      
      if(inliers_num>max_inliers) {
        max_inliers = inliers_num;
        max_T = Tresult;
        best_s = trial_scale;
      }
    }
    Tresult = max_T;
    scale = best_s;
  }

  
   /* Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud,
     * starting with an intial guess
     * Inputs:
     *   source_points
     *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
     *   target_points
     *     The "target" points, i.e., the points to which the source point cloud will be aligned
     *   intial_alignment
     *     An initial estimate of the transformation matrix that aligns the source points to the target points
     *   max_correspondence_distance
     *     A threshold on the distance between any two corresponding points.  Any corresponding points that are further
     *     apart than this threshold will be ignored when computing the source-to-target transformation
     *   outlier_rejection_threshold
     *     A threshold used to define outliers during RANSAC outlier rejection
     *   transformation_epsilon
     *     The smallest iterative transformation allowed before the algorithm is considered to have converged
     *   max_iterations
     *     The maximum number of ICP iterations to perform
     * Return: A transformation matrix that will precisely align the points in source to the points in target
     */
    template <typename PointSource, typename PointTarget>
    Eigen::Matrix4f
    refineAlignment (const typename pcl::PointCloud<PointSource>::Ptr & source_points, const typename pcl::PointCloud<PointTarget>::Ptr & target_points,
                     const Eigen::Matrix4f &initial_alignment, float max_correspondence_distance,
                     float outlier_rejection_threshold, float transformation_epsilon, float euclidean_epsilon, float max_iterations, bool &converged, double &score)
    {
      
      pcl::IterativeClosestPoint<PointSource, PointTarget> icp;
      icp.setMaxCorrespondenceDistance (max_correspondence_distance);
      icp.setRANSACOutlierRejectionThreshold (outlier_rejection_threshold);
      icp.setTransformationEpsilon (transformation_epsilon);
      icp.setEuclideanFitnessEpsilon (euclidean_epsilon);
      icp.setMaximumIterations (max_iterations);
      
      typename pcl::PointCloud<PointSource>::Ptr source_points_transformed (new pcl::PointCloud<PointSource>);
      pcl::transformPointCloud (*source_points, *source_points_transformed, initial_alignment);
      
      icp.setInputCloud (source_points_transformed);
      icp.setInputTarget (target_points);
      
      pcl::PointCloud<PointSource> registration_output;
      icp.align (registration_output);
      
      converged = icp.hasConverged();
      score=icp.getFitnessScore();
      cout<<"ICP has converged:"<<converged << " score: "<<score<<endl;
      
      
      return (icp.getFinalTransformation () * initial_alignment);
    }
    
  
  /* Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud,
   * starting with an intial guess
   * Inputs:
   *   source_points
   *     The "source" points, i.e., the points that must be transformed to align with the target point cloud
   *   target_points
   *     The "target" points, i.e., the points to which the source point cloud will be aligned
   *   transform
   *     An initial estimate of the transformation matrix that aligns the source points to the target points
   *   max_correspondence_distance
   *     A threshold on the distance between any two corresponding points.  Any corresponding points that are further
   *     apart than this threshold will be ignored when computing the source-to-target transformation
   *   translation_threshold
   *     The smallest iterative translation change allowed before the algorithm is considered to have converged
   *   rotation_threshold
   *     The smallest iterative rotation change allowed before the algorithm is considered to have converged
   *   max_iterations
   *     The maximum number of  iterations to perform
   * Note: This method allows more flexibility on the rejection/transformation steps than
   */
  template <typename PointT>
  int
  icp_with_rejection (const typename pcl::PointCloud<PointT>::Ptr &src,
       const typename pcl::PointCloud<PointT>::Ptr &tgt,
       Eigen::Matrix4d &transform,
       int max_iterations, double translation_threshold, double rotation_threshold,
       double max_corrs_distance, bool rejection, double max_normal_angle = 90,
       bool reject_normals = false, bool use_lm = false, bool compute_scale=false)
  {
    CorrespondencesPtr all_correspondences (new Correspondences),
    good_correspondences (new Correspondences);
    
    typename PointCloud<PointT>::Ptr output (new PointCloud<PointT>);
    *output = *src;
    
    Eigen::Matrix4d final_transform (Eigen::Matrix4d::Identity ());
    
    int iterations = 0;
    DefaultConvergenceCriteria<double> icp_convergance (iterations, transform, *good_correspondences);
    
    icp_convergance.setMaximumIterations (max_iterations);
    icp_convergance.setTranslationThreshold (translation_threshold);
    icp_convergance.setRotationThreshold (rotation_threshold);
    
    // ICP loop
    do
    {
      // Find correspondences
      vpcl::correspondance::findCorrespondences<PointT> (output, tgt, *all_correspondences);
      PCL_DEBUG ("Number of correspondences found: %d\n", all_correspondences->size ());
      
      if (rejection)
      {
        // Reject correspondences
        vpcl::correspondance::rejectBadCorrespondences (all_correspondences, output, tgt, *good_correspondences, max_corrs_distance, max_normal_angle, reject_normals);
      }
      else
        *good_correspondences = *all_correspondences;
      
      // Find transformation
      if (use_lm) {
        pcl::registration::TransformationEstimationLM<PointNormal, PointNormal, double> trans_est_lm;
        trans_est_lm.estimateRigidTransformation (*output, *tgt, *good_correspondences, transform);
      }
      else {
        if (compute_scale) {
          vpcl::registration::TransformationEstimationSVD_SRT<PointNormal, PointNormal, double> trans_est_svd;
          trans_est_svd.estimateRigidTransformation (*output, *tgt, *good_correspondences, transform);
        }
        else{
          pcl::registration::TransformationEstimationSVD<PointNormal, PointNormal, double> trans_est_svd;
          trans_est_svd.estimateRigidTransformation (*output, *tgt, *good_correspondences, transform);
        }
      }
          
      // Obtain the final transformation
      final_transform = transform * final_transform;
      
      // Transform the data
      transformPointCloudWithNormals (*src, *output, final_transform.cast<float> ());
      
      // Check if convergence has been reached
      ++iterations;
      
    }
    while (!icp_convergance);
    transform = final_transform;
    return iterations;
  }
  

  
  template <typename PointSource, typename PointTarget>
  double
  refineAlignmentScale (const typename pcl::PointCloud<PointSource>::Ptr & source_points, const typename pcl::PointCloud<PointTarget>::Ptr & target_points,
                        const Eigen::Matrix4f initial_alignment, float max_correspondence_distance,
                        float outlier_rejection_threshold, float transformation_epsilon, float euclidean_epsilon,
                        float max_iterations,int scale_niter, double iteration_scale_step,  double &scale,
                        Eigen::Matrix4f &Tresult)
  {
    
    
    double init_scale = scale;
    Eigen::Matrix4f best_T; double best_scale = scale; double best_score = std::numeric_limits<double>::max(); int max_ninliers = 0;
    
    for(int i=-(scale_niter/2);i<=(scale_niter/2);i++)
    {
      double  trial_scale= (init_scale + (double)i*(iteration_scale_step));
      cout << "Procesing Scale: " << trial_scale << endl;
      Eigen::Matrix4f Tscale = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
      Tscale.topLeftCorner(3,3) *= Eigen::Matrix3f::Identity() * trial_scale;
      
      pcl::PointCloud<pcl::PointNormal>::Ptr source_scaled(new pcl::PointCloud<pcl::PointNormal>);
      pcl::transformPointCloud<pcl::PointNormal>(*source_points, *source_scaled, Tscale);
      
      //correct the translation vector in the initial aligment Tform
      Eigen::Matrix4f initialTformScaled(Eigen::Matrix4f::Identity());
      initialTformScaled = initialTformScaled * initial_alignment;
      cout << "Initial Translation is : \n " << initialTformScaled.block (0, 3, 3, 1) << endl;
      cout << "Initial Translation is : \n " << initial_alignment.block (0, 3, 3, 1) << endl;
      cout << "Trial Scale: " << trial_scale << " Init Scale:" << init_scale << endl;
      initialTformScaled.block (0, 3, 3, 1) *=(trial_scale/init_scale);
      cout << "Adjusted Translation is : \n " << initialTformScaled.block (0, 3, 3, 1) << endl;

      
      
      bool converged = false;
      double fitness_score = 0.0;
      Tresult = refineAlignment<PointSource,PointTarget> (source_scaled, target_points, initialTformScaled,
                                                          max_correspondence_distance,outlier_rejection_threshold,
                                                          transformation_epsilon, euclidean_epsilon, max_iterations, converged, fitness_score);
      
      if (!converged) 
        continue;
      
      int ninliers = 0;
  
//      double fitness_score = vpcl::transform::getFitnessScore<PointSource, PointTarget>(source_scaled, target_points, Tresult, outlier_rejection_threshold/2.0, ninliers);
      cout << "Refine Aligment transform:"<<endl<<Tresult<<endl;
      cout << "Fitness Score: "<<fitness_score<<endl;
      cout << "Number inliers: "<<ninliers<<endl;
      cout << "------------------------------------------------------------------------" << endl;
      
      if(fitness_score < best_score) {
        best_score = fitness_score;
        best_T = Tresult;
        best_scale = trial_scale;
      }
    }
    
    Tresult = best_T;
    scale = best_scale;
    
    cout << "Initial Aligment Best Transform:"<<endl<<Tresult<<endl;
    cout << "Best Fitness Score: "<<best_score<<endl;
    cout << "Best Scale: " << scale <<endl;
    cout << "Max Number inliers: "<<max_ninliers<<endl;

    cout << "------------------------------------------------------------------------" << endl;
    
    
    return best_score;
  }
}


#endif
