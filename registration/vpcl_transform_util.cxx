//:
// \file
// \author Isabel Restrepo
// \date July 30, 2012


#include "vpcl_transform_util.h"
#include "vpcl_corrs_util.h"
#include "vpcl_transformation_estimation_svd_srt.h"


using namespace pcl;
using namespace vpcl;

void transform::computeTransformation (const PointCloud<PointNormal>::Ptr &src,
                                       const PointCloud<PointNormal>::Ptr &tgt,
                                       const PointCloud<FPFHSignature33>::Ptr &fpfhs_src,
                                       const PointCloud<FPFHSignature33>::Ptr &fpfhs_tgt,
                                       float &S,
                                       Eigen::Matrix<double, 3, 3> &R,
                                       Eigen::Matrix<double, 3, 1> &T,
                                       Eigen::Matrix<double, 4, 4> &transform,
                                       CorrespondencesPtr &all_correspondences)
{
  // Find correspondences between keypoints in FPFH space
  //  good_correspondences (new Correspondences);
  correspondance::findCorrespondences<FPFHSignature33> (fpfhs_src, fpfhs_tgt, *all_correspondences);
  
  // Reject correspondences based on their XYZ distance
  //correspondance::rejectBadCorrespondences (all_correspondences, src, tgt, *good_correspondences, max_dist);
  

  std::cerr << "Number of detected correspondances: " << all_correspondences->size() << std::endl;
//  std::cerr << "Number of good correspondances: " << good_correspondences->size() << std::endl;
  
  // Obtain the best transformation between the two sets of keypoints given the remaining correspondences
  vpcl::registration::TransformationEstimationSVD_SRT<PointNormal, PointNormal, double> trans_est;
  trans_est.estimateRigidTransformationSRT (*src, *tgt, *all_correspondences, S, R, T, transform);
}


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
vpgl_perspective_camera<double>
transform::transform_camera(vpgl_perspective_camera<double> const& cam,
                 vgl_rotation_3d<double> const& Rs,
                 vnl_vector_fixed<double, 3> const& ts,
                 const double scale)
{
  vnl_matrix_fixed<double,3,3> Rms = Rs.as_matrix();
  //Get input camera components
  //note, the homogeneous calibration matrix is unaffected by the scale
  vpgl_calibration_matrix<double> K = cam.get_calibration();
  vnl_matrix_fixed<double, 3, 3> R0 = cam.get_rotation().as_matrix();
  vgl_vector_3d<double> tv = cam.get_translation();
  vnl_vector_fixed<double, 3> t0(tv.x(), tv.y(), tv.z());
  //compose rotations
  vnl_matrix_fixed<double, 3, 3> Rt = R0*Rms;
  vgl_rotation_3d<double> Rtr(Rt);
  //compute new translation
  vnl_vector_fixed<double, 3> tt = (1.0/scale)*t0 + R0*ts;
  vgl_vector_3d<double> ttg(tt[0], tt[1], tt[2]);
  //construct transformed camera
  vpgl_perspective_camera<double> camt(K, Rtr, ttg);

  return camt;
}