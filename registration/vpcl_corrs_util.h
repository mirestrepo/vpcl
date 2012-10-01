#ifndef vpcl_corrs_util_h
#define vpcl_corrs_util_h

//:
// \file
// \brief  Utilities for correspondances on clouds form the PVM
// \author Isabel Restrepo
// \date  July 30, 2012
//
// \verbatim
//  Modifications
//   <none yet>
// \endverbatim

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/point_representation.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/boost.h>
#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>


#include <vector.h>
#include <iostream.h>

#include <vul/vul_timer.h>

#include <vnl/vnl_vector_fixed.h>

using namespace pcl;
using namespace std;
namespace vpcl
{
  namespace correspondance
  {    
    // Generic find correspondances
    template <typename PointT>
    void findCorrespondences (const typename PointCloud<PointT>::Ptr &src,
                              const typename PointCloud<PointT>::Ptr &tgt,
                              Correspondences &all_correspondences)
    {
      pcl::registration::CorrespondenceEstimation<PointT, PointT> est;
      est.setInputCloud (src);
      est.setInputTarget (tgt);
      est.determineCorrespondences (all_correspondences);
    }

//#if 0 
    
    template <typename FeatureType>
    void findCorrespondences (const typename pcl::PointCloud<FeatureType>::Ptr &source,
                              const typename pcl::PointCloud<FeatureType>::Ptr &target,
                              std::vector<int>& correspondences,
                              int skip = 1)
    {
      cout << "correspondence assignment..." << std::flush;
      correspondences.resize (source->size());
      
      // Use a KdTree to search for the nearest matches in feature space
      search::KdTree<FeatureType> descriptor_kdtree; // (new pcl::search::KdTree<PointT> ());
      descriptor_kdtree.setInputCloud (target);
      
      // Find the index of the best match for each keypoint, and store it in "correspondences_out"
      const int k = 1;
      std::vector<int> k_indices (k);
      std::vector<float> k_squared_distances (k);
      for (int i = 0; i < static_cast<int> (source->size ()); i=i+skip)
      {
        descriptor_kdtree.nearestKSearch (*source, i, k, k_indices, k_squared_distances);
        correspondences[i] = k_indices[0];
      }
      cout << "OK" << endl;
    }
    
    
    template<typename FeatureType>
    void determineReciprocalCorrespondances (std::vector<int> source2target,
                                             std::vector<int> target2source,
                                             CorrespondencesPtr &correspondences)
    {
      std::vector<std::pair<unsigned, unsigned> > rec_corres;
      for (unsigned cIdx = 0; cIdx < source2target.size (); ++cIdx)
        if (target2source[source2target[cIdx]] == static_cast<int> (cIdx))
          rec_corres.push_back(std::make_pair(cIdx, source2target[cIdx]));
      
      correspondences->resize (rec_corres.size());
      for (unsigned cIdx = 0; cIdx < rec_corres.size(); ++cIdx)
      {
        (*correspondences)[cIdx].index_query = rec_corres[cIdx].first;
        (*correspondences)[cIdx].index_match = rec_corres[cIdx].second;
      }
    }
//#endif

    
    void rejectBadCorrespondences (const CorrespondencesPtr &all_correspondences,
                                   const PointCloud<PointNormal>::Ptr &src,
                                   const PointCloud<PointNormal>::Ptr &tgt,
                                   Correspondences &remaining_correspondences,
                                   double max_dist, double max_angle, bool reject_normals);
    
    
    // Look at descriptor and remove all NAN desciptors and the corresponding point
    template <typename PointT, typename FeatureT, int DIM> void
    removeNaNDescriptors (const  pcl::PointCloud<PointT> &cloud_in,
                          const  pcl::PointCloud<FeatureT> &descriptors_in,
                          pcl::PointCloud<PointT> &cloud_out,
                          pcl::PointCloud<FeatureT> &descriptors_out,
                          std::vector<int> &index)
    {
      // If the clouds are not the same, prepare the output
      if (&cloud_in != &cloud_out)
      {
        cloud_out.header = cloud_in.header;
        cloud_out.points.resize (cloud_in.points.size ());
      }
      if (&descriptors_in != &descriptors_out)
      {
        descriptors_out.header = descriptors_in.header;
        descriptors_out.points.resize (descriptors_in.points.size ());
      }
      
      // Reserve enough space for the indices
      index.resize (cloud_in.points.size ());
      size_t j = 0;   
      cout << "Size of original descriptors: " << cloud_in.points.size() << endl;

      for (size_t i = 0; i <  cloud_in.points.size (); ++i)
      {
        bool is_finite = true;
        for(int d=0;d<DIM;d++)
          is_finite = is_finite && pcl_isfinite(descriptors_in.points[i].descriptor[d]);
  
        if (!is_finite)
          continue;
        
        cloud_out.points[j] = cloud_in.points[i];
        descriptors_out.points[j] = descriptors_in.points[i];
        index[j] = static_cast<int>(i);
        j++;
        
      }
      
      cout << "Size after removing NAN: " << j << endl;
     
      
      if (j != cloud_in.points.size ())
      {
        // Resize to the correct size
        cloud_out.points.resize (j);
        descriptors_out.points.resize (j);
        index.resize (j);
        cloud_out.height = 1;
        cloud_out.width  = static_cast<uint32_t>(j);
        descriptors_out.height = 1;
        descriptors_out.width = static_cast<uint32_t>(j);
      }
      // Removing bad points => dense (note: 'dense' doesn't mean 'organized')
      cloud_out.is_dense = true;
      descriptors_out.is_dense = true;
    }
  }
}


#endif
