//:
// \file
// \author Isabel Restrepo
// \date 7/31/12

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <iostream>
#include <algorithm>
#include <util/vpcl_io_util.h>

using namespace std;

using namespace std;
using namespace sensor_msgs;
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace pcl::visualization;

typedef PointNormal PointT;
typedef PointCloud<PointT> Cloud;
typedef Cloud::Ptr CloudPtr;
typedef Cloud::ConstPtr CloudConstPtr;

boost::shared_ptr<PCLVisualizer> vis;

void
view (const CloudConstPtr &src, const CloudConstPtr &tgt, pcl::CorrespondencesPtr corrs)
{
  // Estimate the centroids of source, target

  Eigen::Matrix3d covariance_matrix;
  Eigen::Vector4d centroid;
  computeMeanAndCovarianceMatrix (*src,
                                  covariance_matrix,
                                  centroid);

  double sigma = sqrt(covariance_matrix.trace());
  
  Eigen::Matrix3f R;
  R.setIdentity();
  R = (1.0/sigma)*R;
  Eigen::Matrix4f tform;
  tform.setIdentity ();
  tform.topLeftCorner (3, 3) = R;
  
  
  cout << "Sigma: " << sigma << "\nCentroid: " << centroid << endl;
//
  // Transform src cloud
  PointCloud<PointNormal>::Ptr cloud_src_demean(new PointCloud<PointNormal>);
  demeanPointCloud (*src, centroid.cast<float>(), *cloud_src_demean);
  PointCloud<PointNormal>::Ptr output_src(new PointCloud<PointNormal>);
  transformPointCloudWithNormals (*cloud_src_demean, *output_src, tform.cast<float>());
  
  // Transform tgt cloud
  PointCloud<PointNormal>::Ptr cloud_tgt_demean(new PointCloud<PointNormal>);
  demeanPointCloud (*tgt, centroid.cast<float>(), *cloud_tgt_demean);
  PointCloud<PointNormal>::Ptr output_tgt(new PointCloud<PointNormal>);
  transformPointCloudWithNormals (*cloud_tgt_demean, *output_tgt, tform.cast<float>());
  
  if (!vis) return;
  PointCloudColorHandlerCustom<PointT> red (output_src, 0, 255, 0);
  PointCloudColorHandlerCustom<PointT> blue (output_tgt, 0, 0, 255);

  if (!vis->updatePointCloud<PointT> (output_src, red, "source"))
  {
    vis->addPointCloud<PointT> (output_src, red, "source");
    vis->resetCameraViewpoint ("source");
  }
  if (!vis->updatePointCloud<PointT> (output_tgt, blue, "target")) vis->addPointCloud<PointT> (output_tgt, blue, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.5, "source");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.5, "target");
 // vis->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, 6, "source");
//  vis->addCoordinateSystem(1.0);
  vis->initCameraParameters ();
//  vis->setBackgroundColor(255, 255, 255);
  TicToc tt;
  tt.tic ();
  
  if(corrs.get() != NULL)
  {
    if (!vis->updateCorrespondences<PointT> (output_src, output_tgt, *corrs, 1))
      vis->addCorrespondences<PointT> (output_src, output_tgt, *corrs, 1, "correspondences");
    tt.toc_print ();
    vis->setShapeRenderingProperties (PCL_VISUALIZER_LINE_WIDTH, 5, "correspondences");
    vis->setShapeRenderingProperties (PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "correspondences");
  }
  vis->spin ();
}

int
main (int argc, char** argv)
{
  vis.reset (new PCLVisualizer ("Registration Visualizer"));
  
  if(argc < 3) {
    cout<<"usage: ./show_clouds cloud1.pcd cloud2.pcd"<<endl;
    return -1;
  }
  
  //Load corrs if applicable
  pcl::CorrespondencesPtr corrs;
  if(argc >= 4) {
//    corrs = util::loadCorrespondences(argv[3]);
//    sort(corrs->begin(), corrs->end(), pcl::isBetterCorrespondence);
//    reverse(corrs->begin(), corrs->end());
  }
  
  //Load input point clouds
  PointCloud<PointNormal>::Ptr src(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud<PointNormal>(argv[1], src);
  PointCloud<PointNormal>::Ptr tgt(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud<PointNormal>(argv[2], tgt);

  cout<<"red Cloud: "<<argv[1]<<endl;
  cout<<"blue Cloud: "<<argv[2]<<endl;
  

  
  // Visualize the results
  view (src, tgt, corrs);
  
}

