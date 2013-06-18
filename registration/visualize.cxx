//:
// \file
// \author Isabel Restrepo
// \date 7/31/12

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/common/eigen.h>
#include <pcl/console/parse.h>
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
view (const CloudConstPtr &src, const CloudConstPtr &tgt, pcl::CorrespondencesPtr corrs, double *bc_color,  double *fcolor1, double *fcolor2, int psize)
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
  PointCloudColorHandlerCustom<PointT> red (output_src, fcolor1[0], fcolor1[1], fcolor1[2]);
  PointCloudColorHandlerCustom<PointT> blue (output_tgt, fcolor2[0], fcolor2[1], fcolor2[2]);

  if (!vis->updatePointCloud<PointT> (output_src, red, "source"))
  {
    vis->addPointCloud<PointT> (output_src, red, "source");
    vis->resetCameraViewpoint ("source");
  }
  if (!vis->updatePointCloud<PointT> (output_tgt, blue, "target")) vis->addPointCloud<PointT> (output_tgt, blue, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.5, "source");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_OPACITY, 0.5, "target");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, psize, "source");
  vis->setPointCloudRenderingProperties (PCL_VISUALIZER_POINT_SIZE, psize, "target");

//  vis->addCoordinateSystem(1.0);
  vis->initCameraParameters ();
  vis->setBackgroundColor(bc_color[0], bc_color[1], bc_color[2]);
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
    cout<<"usage: ./visualize cloud1.pcd cloud2.pcd"<<endl;
    return -1;
  }
  
  cout << "Number of arguments: " << argc << endl;
  
  // Command line parsing
  double bcolor[3] = {0, 0, 0};
  pcl::console::parse_3x_arguments (argc, argv, "-bc", bcolor[0], bcolor[1], bcolor[2], true);
  
  double fcolor1[3] = {0, 255, 0};
  pcl::console::parse_3x_arguments (argc, argv, "-fc1", fcolor1[0], fcolor1[1], fcolor1[2]);
  
  double fcolor2[3] = {0, 0, 255};
  pcl::console::parse_3x_arguments (argc, argv, "-fc2", fcolor2[0], fcolor2[1], fcolor2[2]);
  
  int psize;
  pcl::console::parse_argument (argc, argv, "-ps", psize);
  
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
  view (src, tgt, corrs, bcolor, fcolor1, fcolor2, psize);
  
}

