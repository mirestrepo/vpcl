"""
Created on April 17, 2012

@author:Isabel Restrepo

Python bindings for pcl related processes
"""

from dbrec3d_register import dbrec3d_batch, dbvalue; 
import os

#********************************************************************
#  Compute Descriptor for Local Shape Features
#********************************************************************
def compute_descriptor(file_in, file_out, radius, feature_type, jobs=8, point_type="PointNormal"):
  dbrec3d_batch.init_process("pcl_compute_descriptor_process");
  dbrec3d_batch.set_input_string(0,file_in);
  dbrec3d_batch.set_input_string(1,point_type);
  dbrec3d_batch.set_input_string(2,file_out);
  dbrec3d_batch.set_input_string(3,feature_type);
  dbrec3d_batch.set_input_double(4,radius);
  dbrec3d_batch.set_input_int(5, jobs);
  dbrec3d_batch.run_process();
  filters = dbvalue(id, type);
  return filters;

#********************************************************************
#  Compute Spin Image Descriptor
#********************************************************************
def compute_spin_image(file_in, file_out, radius, width=8, sup_angle=0.5, num_neighbors=8, radial=False, angular=False):
  dbrec3d_batch.init_process("pcl_compute_spin_image_process");
  dbrec3d_batch.set_input_string(0, file_in);
  dbrec3d_batch.set_input_double(1, radius);
  dbrec3d_batch.set_input_int(2, width);
  dbrec3d_batch.set_input_double(3, sup_angle);
  dbrec3d_batch.set_input_int(4, num_neighbors);
  dbrec3d_batch.set_input_bool(5, radial);
  dbrec3d_batch.set_input_bool(6, angular);
  dbrec3d_batch.set_input_string(7, file_out);
  return dbrec3d_batch.run_process();
    
