//:
// \brief
// \file
// \author Isabel Restrepo
// \date Jan 25, 2012

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_parameters.h>

#include <brdb/brdb_value.h>

#include <util/vpcl_features_util.h>
#include <util/vpcl_io_util.h>

#include <pcl/point_types.h>
#include <pcl/features/3dsc.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <vul/vul_file.h>

#include "pcl/impl/instantiate.hpp"
#include "pcl/features/spin_image.h"
#include "pcl/features/impl/spin_image.hpp"
#include <pcl/search/impl/organized.hpp>

#define SPIN_DIM 153

POINT_CLOUD_REGISTER_POINT_STRUCT (pcl::Histogram<SPIN_DIM>,(float[SPIN_DIM], histogram, spinimage));
PCL_INSTANTIATE_PRODUCT(SpinImageEstimation, (PCL_XYZ_POINT_TYPES)(PCL_NORMAL_POINT_TYPES)((pcl::Histogram<SPIN_DIM>)));
PCL_INSTANTIATE(KdTree, PCL_POINT_TYPES)
typedef pcl::PointNormal  NORMAL;
PCL_INSTANTIATE(OrganizedNeighbor, PCL_XYZ_POINT_TYPES)


//:global variables
namespace vpcl_compute_spin_image_process_globals 
{
  using namespace pcl;
  using namespace pcl::io;
  
  const unsigned n_inputs_ = 8;
  const unsigned n_outputs_ = 0;
  
  void
  print_help ()
  {
    vcl_cout << "Syntax is:"
    << "  input0 - input_cloud: use a radius of Xm around each point to determine the neighborhood\n"
    << "  input1 - radius: use a radius of Xm around each point to determine the neighborhood (this is the bin size - a good number is 4* mesh resolution?)\n"
    << "  input2 - width: resolution (width) of a spin-image \n"
    << "  input3 - suppangle: min cosine of support angle for filtering points by normals \n"
    << "  input4 - neigh: min number of neighbours to compute a spin-image \n "
    << "  input5 - radial: boolean toggles radial structure of a spin-image\n"
    << "  input6 - angular: toggles angular domain of a spin-image"
    << "  input7 - cloud_out: path to point cloud containing descriptors" << vcl_endl;
  }
              
}


//:sets input and output types
bool vpcl_compute_spin_image_process_cons(bprb_func_process& pro)
{
  using namespace vpcl_compute_spin_image_process_globals ;
  
  vcl_vector<vcl_string> input_types_(n_inputs_);
  unsigned i = 0;
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "double";
  input_types_[i++] = "int";
  input_types_[i++] = "double";
  input_types_[i++] = "int";
  input_types_[i++] = "bool";
  input_types_[i++] = "bool";
  input_types_[i++] = "vcl_string"; 
  
  vcl_vector<vcl_string> output_types_(n_outputs_);
  
  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}


//:the process
bool vpcl_compute_spin_image_process(bprb_func_process& pro)
{
  using namespace vpcl_compute_spin_image_process_globals;
  using namespace pcl;

  //get inputs
  unsigned i = 0;
  vcl_string cloud_in_file = pro.get_input<vcl_string>(i++);
  double radius = pro.get_input<double>(i++);
  int width = pro.get_input<int>(i++);
  double sup_angle = pro.get_input<double>(i++);
  int num_neighbors = pro.get_input<int>(i++);
  bool radial = pro.get_input<bool>(i++);
  bool angular = pro.get_input<bool>(i++);
  vcl_string cloud_out_file = pro.get_input<vcl_string>(i++);


  //create and read from file the input point cloud
  PointCloud<PointNormal>::Ptr cloud_in(new PointCloud<PointNormal>);
  vpcl_io_util::load_cloud<PointNormal>(cloud_in_file, cloud_in);

  PointCloud<Histogram<SPIN_DIM> >::Ptr cloud_out(new PointCloud<Histogram<SPIN_DIM> >);

  //compute the features
  vcl_cout << "Computing Signatures: " <<vcl_endl;
  vul_timer timer;
  timer.mark();
  vpcl_compute_spin_images<SPIN_DIM>(cloud_in, cloud_out, radius, width, sup_angle, num_neighbors, radial, angular);
  
  //save ouput point cloud to file
  vpcl_io_util::save_histogram_as_txt<Histogram<SPIN_DIM>, SPIN_DIM>(cloud_out_file, cloud_out);
  //pcl::io::savePCDFileASCII (cloud_out_file, *cloud_out); 
  // Display and retrieve the spin image descriptor vector for the first point.
  pcl::Histogram<SPIN_DIM> first_descriptor = cloud_out->points[0];
  vcl_cout << first_descriptor << vcl_endl;
  
  vcl_string log_ext("_time_log.log");
  vcl_ofstream ofs;
  ofs.open((vul_file::strip_extension(cloud_out_file.c_str()) + log_ext).c_str());
  timer.print(ofs); 
  ofs.close();

  return true;
  
}