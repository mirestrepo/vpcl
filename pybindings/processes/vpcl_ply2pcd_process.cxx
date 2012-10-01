//:
// \file
// \brief A process to export .ply point cloud to .pcd format
// \author Isabel Restrepo
// \date 9/18/12

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_parameters.h>

#include <brdb/brdb_value.h>

#include <pcl/point_types.h>

#include <util/vpcl_io_util.h>

using namespace std;

//:global variables
namespace vpcl_ply2pcd_process_globals 
{
  const unsigned n_inputs_ = 4;
  const unsigned n_outputs_ = 0;
}


//:sets input and output types
bool vpcl_ply2pcd_process_cons(bprb_func_process& pro)
{
  using namespace vpcl_ply2pcd_process_globals ;
  
  vcl_vector<vcl_string> input_types_(n_inputs_);
  unsigned i =0;
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "vcl_string";
  input_types_[i++] = "bool";

  vcl_vector<string> output_types_(n_outputs_);
  
  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}


//:the process
bool vpcl_ply2pcd_process(bprb_func_process& pro)
{
  using namespace vpcl_ply2pcd_process_globals;
  using namespace pcl;
  
  //get inputs
  unsigned i =0;
  string ply_file = pro.get_input<vcl_string>(i++);
  string pcd_file = pro.get_input<vcl_string>(i++);
  string point_type = pro.get_input<vcl_string>(i++);
  bool binary_mode = pro.get_input<bool>(i++);
  
  //create and read from file the input point cloud
  if (point_type == "PointNormal") {
    PointCloud<PointNormal>::Ptr cloud(new PointCloud<PointNormal>);
    vpcl_io_util::load_cloud<PointNormal>(ply_file, cloud);
    string file_type = vul_file::extension(pcd_file);
    if (file_type == ".pcd") {
      if (pcl::io::savePCDFile (pcd_file, *cloud, binary_mode) < 0)
        return (false);
    }
    else {
      cout << "Output file type not supported: " << file_type << endl;
      return false;
    }
  }
  else if (point_type == "PointXYZ") {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    vpcl_io_util::load_cloud<PointXYZ>(ply_file, cloud);
    string file_type = vul_file::extension(pcd_file);
    if (file_type == ".pcd") {
      if (pcl::io::savePCDFile (pcd_file, *cloud, binary_mode) < 0)
        return (false);
    }
    else {
      cout << "Output file type not supported: " << file_type << endl;
      return false;
    }
  }
  else {
    vcl_cerr << "Unsupported Feature or Point Types" <<vcl_endl;
    
    return false;
  }


  
  
  return true;
}