/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010-2011, Willow Garage, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 *
 */
#ifndef VPCL_TRANSFORMATION_ESTIMATION_SVD_SRT_H_
#define VPCL_TRANSFORMATION_ESTIMATION_SVD_SRT_H_

#include <pcl/registration/transformation_estimation_svd_scale.h>

namespace vpcl
{
  namespace registration
  {
    /** @b TransformationEstimationSVD_SRT implements SVD-based estimation of
     * the transformation aligning the given correspondences.
     * Optionally the scale is estimated. Note that the similarity transform might not be optimal for the underlying Frobenius Norm.
     *
     * \note The class is templated on the source and target point types as well as on the output scalar of the transformation matrix (i.e., float or double). Default: float.
     * \author Suat Gedikli
     * \modifications Isabel Restrepo : Return the components of the transformation i.e S, R T
     */
    using namespace pcl::registration;

    template <typename PointSource, typename PointTarget, typename Scalar = float>
    class TransformationEstimationSVD_SRT : public TransformationEstimationSVDScale<PointSource, PointTarget, Scalar>
    {
    public:
      typedef typename TransformationEstimationSVD_SRT<PointSource, PointTarget, Scalar>::Matrix4 Matrix4;
      
      void
      estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                                   const pcl::PointCloud<PointTarget> &cloud_tgt,
                                   const pcl::Correspondences &correspondences,
                                   float &S,
                                   Eigen::Matrix<Scalar, 3, 3> &R,
                                   Eigen::Matrix<Scalar, 3, 1> &T,
                                   Eigen::Matrix<Scalar, 4, 4> &transformation_matrix) const;

      
      
    protected:
      /** \brief Obtain a 4x4 rigid transformation matrix from a correlation matrix H = src * tgt'
       * \param[in] cloud_src_demean the input source cloud, demeaned, in Eigen format
       * \param[in] centroid_src the input source centroid, in Eigen format
       * \param[in] cloud_tgt_demean the input target cloud, demeaned, in Eigen format
       * \param[in] centroid_tgt the input target cloud, in Eigen format
       * \param[out] S the resultant Scale
       * \param[out] R the resultant rotation matrix
       * \param[out] T the resultant translation vector
       * \param[out] transformation_matrix the resultant 4x4 rigid transformation matrix
       */
      void
      getTransformationFromCorrelation (const Eigen::MatrixXf &cloud_src_demean,
                                        const Eigen::Vector4f &centroid_src,
                                        const Eigen::MatrixXf &cloud_tgt_demean,
                                        const Eigen::Vector4f &centroid_tgt,
                                        float &S,
                                        Eigen::Matrix<Scalar, 3, 3> &R,
                                        Eigen::Matrix<Scalar, 3, 1> &T,
                                        Eigen::Matrix<Scalar, 4, 4> &transformation_matrix) const;
    };
    
  }
}

using namespace vpcl::registration;

//////////////////////////////////////////////////////////////////////////////////////////////
template <typename PointSource, typename PointTarget, typename Scalar>
void TransformationEstimationSVD_SRT<PointSource, PointTarget, Scalar>::estimateRigidTransformation (const pcl::PointCloud<PointSource> &cloud_src,
                                                                                                       const pcl::PointCloud<PointTarget> &cloud_tgt,
                                                                                                       const pcl::Correspondences &correspondences,
                                                                                                       float &S,
                                                                                                       Eigen::Matrix<Scalar, 3, 3> &R,
                                                                                                       Eigen::Matrix<Scalar, 3, 1> &T,
                                                                                                       Eigen::Matrix<Scalar, 4, 4> &transformation_matrix) const
{
  std::vector<int> indices_src, indices_tgt;
  pcl::registration::getQueryIndices (correspondences, indices_src);
  pcl::registration::getMatchIndices (correspondences, indices_tgt);
  
  // <cloud_src,cloud_src> is the source dataset
  Eigen::Vector4f centroid_src, centroid_tgt;
  // Estimate the centroids of source, target
  compute3DCentroid (cloud_src, indices_src, centroid_src);
  compute3DCentroid (cloud_tgt, indices_tgt, centroid_tgt);
  
  // Subtract the centroids from source, target
  Eigen::MatrixXf cloud_src_demean;
  demeanPointCloud (cloud_src, indices_src, centroid_src, cloud_src_demean);
  
  Eigen::MatrixXf cloud_tgt_demean;
  demeanPointCloud (cloud_tgt, indices_tgt, centroid_tgt, cloud_tgt_demean);
  
  getTransformationFromCorrelation (cloud_src_demean, centroid_src, cloud_tgt_demean, centroid_tgt, S, R, T, transformation_matrix);
}

template <typename PointSource, typename PointTarget, typename Scalar>
void TransformationEstimationSVD_SRT<PointSource, PointTarget, Scalar>::getTransformationFromCorrelation (const Eigen::MatrixXf &cloud_src_demean,
                                                                                                          const Eigen::Vector4f &centroid_src,
                                                                                                          const Eigen::MatrixXf &cloud_tgt_demean,
                                                                                                          const Eigen::Vector4f &centroid_tgt,
                                                                                                          float &S,
                                                                                                          Eigen::Matrix<Scalar, 3, 3> &R,
                                                                                                          Eigen::Matrix<Scalar, 3, 1> &T,
                                                                                                          Eigen::Matrix<Scalar, 4, 4> &transformation_matrix) const
{
  transformation_matrix.setIdentity ();
  
  // Assemble the correlation matrix H = source * target'
  Eigen::Matrix<Scalar, 3, 3> H = (cloud_src_demean.cast<Scalar> () * cloud_tgt_demean.cast<Scalar> ().transpose ()).topLeftCorner (3, 3);
  
  // Compute the Singular Value Decomposition
  Eigen::JacobiSVD<Eigen::Matrix<Scalar, 3, 3> > svd (H, Eigen::ComputeFullU | Eigen::ComputeFullV);
  Eigen::Matrix<Scalar, 3, 3> u = svd.matrixU ();
  Eigen::Matrix<Scalar, 3, 3> v = svd.matrixV ();
  
  // Compute R = V * U'
  if (u.determinant () * v.determinant () < 0)
  {
    for (int x = 0; x < 3; ++x)
      v (x, 2) *= -1;
  }
  
  Eigen::Matrix<Scalar, 3, 3> R2 = v * u.transpose ();

  R = v * u.transpose ();
  
  // rotated cloud
//  Eigen::Matrix<Scalar, Eigen::Dynamic, Eigen::Dynamic> src_ = R2 * cloud_src_demean.cast<Scalar> ();
  
  float scale1;
  double sum_ss = 0.0f, sum_tt = 0.0f, sum_tt_ = 0.0f;
  for (unsigned corrIdx = 0; corrIdx < cloud_src_demean.cols (); ++corrIdx)
  {
    sum_ss += cloud_src_demean (0, corrIdx) * cloud_src_demean (0, corrIdx);
    sum_ss += cloud_src_demean (1, corrIdx) * cloud_src_demean (1, corrIdx);
    sum_ss += cloud_src_demean (2, corrIdx) * cloud_src_demean (2, corrIdx);
    
    sum_tt += cloud_tgt_demean (0, corrIdx) * cloud_tgt_demean (0, corrIdx);
    sum_tt += cloud_tgt_demean (1, corrIdx) * cloud_tgt_demean (1, corrIdx);
    sum_tt += cloud_tgt_demean (2, corrIdx) * cloud_tgt_demean (2, corrIdx);
    
//    sum_tt_ += cloud_tgt_demean (0, corrIdx) * src_ (0, corrIdx);
//    sum_tt_ += cloud_tgt_demean (1, corrIdx) * src_ (1, corrIdx);
//    sum_tt_ += cloud_tgt_demean (2, corrIdx) * src_ (2, corrIdx);
  }
  
  scale1 = sqrt (sum_tt / sum_ss); //Forbenius norm
//  scale2 = sum_tt_ / sum_ss; // Why is this used in PCL instead of Forbenius norm? I think this is equivalent to S^2/S...but I'm not sure
                               // what's the computational difference...
  S=scale1;
  transformation_matrix.topLeftCorner (3, 3) = S * R;
  const Eigen::Matrix<Scalar, 3, 1> Rc (R * centroid_src.cast<Scalar> ().head (3));
  // Xtgt =  scale*(rotation*Xsrc + t) --> Ctgt = S*(RCsrc +t) --> t = (1/S)Ctgt - RCsrc
//  T = (1.0f/S) * centroid_tgt.cast<Scalar> (). head (3) - Rc;
  T = (1.0f/S) * centroid_tgt.cast<Scalar> (). head (3)  -  R * centroid_src.cast<Scalar> ().head (3);
  cout << "Centroid src: " << centroid_src.cast<Scalar> ().head (3)<<  "\nCentroid tgt: " << centroid_tgt.cast<Scalar> (). head (3) << endl;
  transformation_matrix.block (0, 3, 3, 1) = S*T;
  cout << S << '\n'
  << R << '\n'
  << T << '\n';
}


#endif /* VPCL_TRANSFORMATION_ESTIMATION_SVD_SRT_H_ */

