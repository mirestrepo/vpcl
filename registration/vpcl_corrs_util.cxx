//:
// \file
// \author Isabel Restrepo
// \date July 30, 2012

#include "vpcl_corrs_util.h"

#include <pcl/search/kdtree.h> 
#include <pcl/registration/correspondence_rejection_distance.h>


using namespace vpcl;
using namespace pcl;



void correspondance::rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
                                               const PointCloud<PointNormal>::Ptr &src,
                                               const PointCloud<PointNormal>::Ptr &tgt,
                                               Correspondences &remaining_correspondences,
                                               float max_dist)
{
  pcl::registration::CorrespondenceRejectorDistance rej;
  rej.setInputCloud<PointNormal> (src);
  rej.setInputTarget<PointNormal> (tgt);
  rej.setMaximumDistance (max_dist);  
  rej.setInputCorrespondences (all_correspondences);
  rej.getCorrespondences (remaining_correspondences);
} 