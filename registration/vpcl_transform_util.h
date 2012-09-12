#ifndef vpcl_transform_util_h
#define vpcl_transform_util_h

//:
// \file
// \brief  Utilities for rigid transfomations on clouds form the PVM
// \author Isabel Restrepo
// \date  July 30, 2012
//
// \verbatim
//  Modifications
//   <none yet>
// \endverbatim

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/registration/correspondence_estimation.h>

#include <vul/vul_timer.h>
#include <vnl/vnl_vector_fixed.h>
#include <vpgl/vpgl_perspective_camera.h>

#include <iostream>
using namespace std;

using namespace pcl;

namespace vpcl
{
  namespace transform
  {
    void computeTransformation (const PointCloud<PointNormal>::Ptr &src,
                                const PointCloud<PointNormal>::Ptr &tgt,
                                const PointCloud<FPFHSignature33>::Ptr &fpfhs_src,
                                const PointCloud<FPFHSignature33>::Ptr &fpfhs_tgt,
                                float &S,
                                Eigen::Matrix<double, 3, 3> &R,
                                Eigen::Matrix<double, 3, 1> &T,
                                Eigen::Matrix<double, 4, 4> &transform,
                                CorrespondencesPtr &all_correspondences);
   
    // The input camera is in the coordinate system of pts0. The output camera
    // is mapped to the coordinate system of pts1, that is,
    //
    //  x1 =  K[R0|t0](Hs Hs^-1) X1, where Hs is the similarity transform, s.t X1 = Hs*X0
    //
    // Thus, the similarity transform is applied to the camera as,
    // (s = scale)
    //                        _     _  _      _
    //                       |s 0 0 0||        |
    //  K[R' | t'] = K[R0|t0]|0 s 0 0||  Rs  ts|
    //                       |0 0 s 0||        |
    //                       |0 0 0 1|| 0 0 0 1|
    //                        -      - -      -
    // It follows that R' = R0*Rs and t' = t0/s + R0*ts
    //
    vpgl_perspective_camera<double>
    transform_camera(vpgl_perspective_camera<double> const& cam,
                     vgl_rotation_3d<double> const& Rs,
                     vnl_vector_fixed<double, 3> const& ts,
                     const double scale);
    
    template <typename PointSource, typename PointTarget> double
    getFitnessScore (const typename PointCloud<PointSource>::Ptr &src,
                     const typename PointCloud<PointTarget>::Ptr &tgt,
                     const Eigen::Matrix4f Tform,
                     double outlier_threshold, int &ninliers)
    {
      double fitness_score = 0.0;
      
      // Transform the input dataset using the final transformation
      PointCloud<PointSource> src_transformed;
      transformPointCloud (*src, src_transformed, Tform);
      
      std::vector<int> nn_indices (1);
      std::vector<float> nn_dists (1);
      
      // For each point in the source dataset
      ninliers = 0;
      for (size_t i = 0; i < src_transformed.points.size (); ++i)
      {
        Eigen::Vector4f p1 = Eigen::Vector4f (src_transformed.points[i].x,
                                              src_transformed.points[i].y,
                                              src_transformed.points[i].z, 0);
        
        // Find its nearest neighbor in the target
        pcl::KdTreeFLANN<PointTarget> tree (true);
        
        tree.setInputCloud (tgt);

        tree.nearestKSearch (src_transformed.points[i], 1, nn_indices, nn_dists);
        
        // Deal with noise, occlussions...
        if (nn_dists[0] > outlier_threshold)
          continue;
        
        Eigen::Vector4f p2 = Eigen::Vector4f (tgt->points[nn_indices[0]].x,
                                              tgt->points[nn_indices[0]].y,
                                              tgt->points[nn_indices[0]].z, 0);
        // Calculate the fitness score
        fitness_score += fabs ((p1-p2).squaredNorm ());
        ninliers++;
      }
      
      if (ninliers > 0)
        return (fitness_score / ninliers);
      else
        return (std::numeric_limits<double>::max ());
    }
    
    
  }
}


#endif
