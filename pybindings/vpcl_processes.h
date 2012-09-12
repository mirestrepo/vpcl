#ifndef vpcl_processes_h_
#define vpcl_processes_h_

#include <bprb/bprb_func_process.h>
#include <bprb/bprb_macros.h>

DECLARE_FUNC_CONS(vpcl_compute_spin_image_process);
DECLARE_FUNC_CONS(vpcl_compute_descriptor_process);
DECLARE_FUNC_CONS(vpcl_feature_based_rigid_transform_process);
DECLARE_FUNC_CONS(vpcl_register_ia_process);
DECLARE_FUNC_CONS(vpcl_register_icp_process);

#endif
