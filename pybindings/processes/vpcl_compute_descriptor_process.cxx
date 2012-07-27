//:
// \brief
// \file
// \author Isabel Restrepo
// \date 11/8/11

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_parameters.h>

#include <brdb/brdb_value.h>

#include <pcl/point_types.h>
#include <pcl/features/fpfh_omp.h>

#include <pcl/features/3dsc.h>
#include <pcl/features/shot_omp.h>

#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>

#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <util/vpcl_features_util.h>
#include <util/vpcl_io_util.h>

#include <vul/vul_file.h>
#include <vul/vul_timer.h>

#include <pcl/features/impl/3dsc.hpp>


//:global variables
namespace vpcl_compute_descriptor_process_globals 
{
  using namespace pcl;
  using namespace pcl::io;
  
  const unsigned n_inputs_ = 6;
  const unsigned n_outputs_ = 0;
  
  void
  print_help ()
  {
    vcl_cout << "Syntax is:"
    << "  input0 - Path input_cloud\n"
    << "  input1 - String identifying point type of the input cloud e.g PointNormal\n"
    << "  input2 - Path onput_cloud\n"
    << "  input3 - String identifying the type of descriptor e.g FPFH, SHOT, ShapeContext\n"
    << "  input4 - radius: use a radius of Xm around each point to determine the neighborhood (this is the bin size - a good number is 4* mesh resolution?)\n"
    << vcl_endl;
  }
}


//:sets input and output types
bool vpcl_compute_descriptor_process_cons(bprb_func_process& pro)
{
  using namespace vpcl_compute_descriptor_process_globals ;
  
  vcl_vector<vcl_string> input_types_(n_inputs_);
  unsigned i = 0;
  input_types_[i++] = "vcl_string"; //Point cloud input path
  input_types_[i++] = "vcl_string"; //Input cloud point type
  input_types_[i++] = "vcl_string"; //Output Cloud path
  input_types_[i++] = "vcl_string"; //Descriptor Type
  input_types_[i++] = "double";  //radius for neighborhood computation
  input_types_[i++] = "int"; //number of jobs
  
  vcl_vector<vcl_string> output_types_(n_outputs_);
  
  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}


//:the process
bool vpcl_compute_descriptor_process(bprb_func_process& pro)
{
  using namespace vpcl_compute_descriptor_process_globals;
  using namespace pcl;

  
  if ( pro.n_inputs() < n_inputs_ ) {
    vcl_cout << pro.name() << ": The input number should be " << n_inputs_<< vcl_endl;
    print_help();
    return false;
  }
  
  //get inputs
  unsigned i = 0;
  vcl_string cloud_in_file = pro.get_input<vcl_string>(i++);
  vcl_string point_type = pro.get_input<vcl_string>(i++);
  vcl_string cloud_out_file = pro.get_input<vcl_string>(i++);
  vcl_string descriptor_type = pro.get_input<vcl_string>(i++);
  double radius = pro.get_input<double>(i++);
  int jobs =1;
  jobs = pro.get_input<int>(i++);
  
  vcl_cout << "Number of jobs: " << jobs << vcl_endl;
  
  if (point_type == "PointNormal") {
    //create and read from file the input point cloud
    PointCloud<PointNormal>::Ptr cloud_in(new PointCloud<PointNormal>);
    vpcl_io_util::load_cloud(cloud_in_file, cloud_in);
    if(descriptor_type == "FPFH")
    {
      //Estimation
      typedef FPFHSignature33 FeatureType;
      typedef PointNormal PointType;
      typedef PointNormal NormalType;
      FPFHEstimationOMP<PointType, NormalType, FeatureType>::Ptr extractor(new FPFHEstimationOMP<PointType, NormalType, FeatureType>(jobs));

      extractor->setInputCloud(cloud_in);
      extractor->setInputNormals(cloud_in);
      extractor->setSearchMethod(search::KdTree<PointType>::Ptr (new search::KdTree<PointType>));
      extractor->setRadiusSearch(radius);
      PointCloud<FeatureType>::Ptr descriptors(new PointCloud<FeatureType>);
      vul_timer timer;
      timer.mark();
      vcl_cout << "Descriptor extraction FPFH..." << vcl_endl;
      extractor->compute (*descriptors);
      vcl_cout << "Done \n";
      //save ouput point cloud to file
      vpcl_io_util::save_histogram_as_txt<FeatureType, 33>(cloud_out_file, descriptors);    
      vcl_string log_ext("_time_log.log");
      vcl_ofstream ofs;
      ofs.open((vul_file::strip_extension(cloud_out_file.c_str()) + log_ext).c_str());
      timer.print(ofs); 
      ofs.close();
      return true;
    }
    else if (descriptor_type == "SHOT")
    {
      typedef SHOT FeatureType;
      typedef PointNormal PointType;
      typedef PointNormal NormalType;
      SHOTEstimationOMP<PointType, NormalType, FeatureType>::Ptr extractor(new SHOTEstimationOMP<PointType, NormalType, FeatureType>(jobs));
      extractor->setInputCloud(cloud_in);
      extractor->setInputNormals(cloud_in);
      extractor->setSearchMethod(search::KdTree<PointType>::Ptr (new search::KdTree<PointType>));
      extractor->setRadiusSearch(radius);
      PointCloud<FeatureType>::Ptr descriptors(new PointCloud<FeatureType>);
      vul_timer timer;
      timer.mark();
      vcl_cout << "Descriptor extraction SHOT..." << vcl_endl;
      extractor->compute (*descriptors);
      vcl_cout << "Done \n";
      timer.print(vcl_cout);       //save ouput point cloud to file
      vpcl_io_util::save_descriptors_as_txt<FeatureType>(cloud_out_file, descriptors);
      vcl_string log_ext("_time_log.log");
      vcl_ofstream ofs;
      ofs.open((vul_file::strip_extension(cloud_out_file.c_str()) + log_ext).c_str());
      timer.print(ofs); 
      ofs.close();
      return true;
    }else if (descriptor_type == "ShapeContext")
    {
      typedef ShapeContext FeatureType;
      typedef PointNormal PointType;
      typedef PointNormal NormalType;
      ShapeContext3DEstimation<PointType, NormalType, FeatureType> extractor;
      extractor.setInputCloud(cloud_in);
      extractor.setInputNormals(cloud_in);
      extractor.setSearchMethod(search::KdTree<PointType>::Ptr (new search::KdTree<PointType>));
      extractor.setRadiusSearch(radius);
      extractor.setPointDensityRadius (radius/5.0);
      extractor.setMinimalRadius(radius/10.0);
      PointCloud<FeatureType>::Ptr descriptors(new PointCloud<FeatureType>);
      vul_timer timer;
      timer.mark();
      vcl_cout << "Descriptor extraction ShapeContext..." << vcl_endl;
      extractor.compute (*descriptors);
      vcl_cout << "Done \n";
      timer.print(vcl_cout);       //save ouput point cloud to file
      vpcl_io_util::save_descriptors_as_txt<FeatureType>(cloud_out_file, descriptors);
      vcl_string log_ext("_time_log.log");
      vcl_ofstream ofs;
      ofs.open((vul_file::strip_extension(cloud_out_file.c_str()) + log_ext).c_str());
      timer.print(ofs);
      ofs.close();
      return true;
    }
  }
 
  vcl_cerr << "Unsupported Feature or Point Types" <<vcl_endl;
  
  return false;
}