//:
// \brief
// \file
// \author Isabel Restrepo
// \date 9/18/12

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_parameters.h>

#include <brdb/brdb_value.h>


//:global variables
namespace vpcl_ply2pcd_process_globals 
{
  const unsigned n_inputs_ = ;
  const unsigned n_outputs_ =;
}


//:sets input and output types
bool vpcl_ply2pcd_process_cons(bprb_func_process& pro)
{
  using namespace vpcl_ply2pcd_process_globals ;
  
  vcl_vector<vcl_string> input_types_(n_inputs_);
  input_types_[0] = ;
  
  vcl_vector<vcl_string> output_types_(n_outputs_);
  output_types_[0] = ;
  
  return pro.set_input_types(input_types_) && pro.set_output_types(output_types_);
}


//:the process
bool vpcl_ply2pcd_process(bprb_func_process& pro)
{
  using namespace vpcl_ply2pcd_process_globals;
  
  //get inputs
  vcl_string fname = pro.get_input<vcl_string>(0);
  
  
  //store output
  pro.set_output_val<boxm_scene_base_sptr>(0, output);
  
  return true;
}