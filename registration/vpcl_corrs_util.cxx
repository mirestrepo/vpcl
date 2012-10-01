//:
// \file
// \author Isabel Restrepo
// \date July 30, 2012

#include "vpcl_corrs_util.h"

#include <pcl/common/angles.h>
#include <pcl/search/kdtree.h> 
#include <pcl/registration/correspondence_rejection_distance.h>
#include <pcl/registration/correspondence_rejection_surface_normal.h>


using namespace vpcl;
using namespace pcl;



void correspondance::rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
                                               const PointCloud<PointNormal>::Ptr &src,
                                               const PointCloud<PointNormal>::Ptr &tgt,
                                               Correspondences &remaining_correspondences,
                                               double max_dist, double max_angle, bool reject_normals)
{
  pcl::registration::CorrespondenceRejectorDistance rej;
  rej.setInputCloud<PointNormal> (src);
  rej.setInputTarget<PointNormal> (tgt);
  rej.setMaximumDistance (max_dist);  
  rej.setInputCorrespondences (all_correspondences);
  
  if(reject_normals)
  {
    PCL_DEBUG ( "Rejecting points with far away surface normals\n" );
    CorrespondencesPtr remaining_correspondences_temp (new Correspondences);
    rej.getCorrespondences (*remaining_correspondences_temp);
    
    PCL_DEBUG ("[rejectBadCorrespondences - points] Number of correspondences remaining after rejection: %d\n", remaining_correspondences_temp->size ());
    
    // Reject if the angle between the normals is really off
    pcl::registration::CorrespondenceRejectorSurfaceNormal rej_normals;
    rej_normals.setThreshold (cos(pcl::deg2rad (max_angle)));
    rej_normals.initializeDataContainer<PointNormal, PointNormal> ();
    rej_normals.setInputCloud<PointNormal> (src);
    rej_normals.setInputNormals<PointNormal, PointNormal> (src);
    rej_normals.setInputTarget<PointNormal> (tgt);
    rej_normals.setTargetNormals<PointNormal, PointNormal> (tgt);
    rej_normals.setInputCorrespondences (remaining_correspondences_temp);
    rej_normals.getCorrespondences (remaining_correspondences);
    
    PCL_DEBUG ("[rejectBadCorrespondences - normals] Number of correspondences remaining after rejection: %d\n", remaining_correspondences.size ());
  }else{  
  rej.getCorrespondences (remaining_correspondences);
  PCL_DEBUG ("[rejectBadCorrespondences - points] Number of correspondences remaining after rejection: %d\n", remaining_correspondences.size ());
  }
}

