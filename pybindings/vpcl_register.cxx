#include "vpcl_register.h"

#include <bprb/bprb_macros.h>
#include <bprb/bprb_batch_process_manager.h>
#include <bprb/bprb_func_process.h>

#include "vpcl_processes.h"

void vpcl_register::register_datatype()
{

}

void vpcl_register::register_process()
{
  REG_PROCESS_FUNC_CONS2(vpcl_compute_spin_image_process);
  REG_PROCESS_FUNC_CONS2(vpcl_compute_descriptor_process);
  REG_PROCESS_FUNC_CONS2(vpcl_feature_based_rigid_transform_process);
  REG_PROCESS_FUNC_CONS2(vpcl_register_ia_process);
  REG_PROCESS_FUNC_CONS2(vpcl_register_icp_process);
} 

