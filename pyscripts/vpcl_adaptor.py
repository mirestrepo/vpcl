"""
Created on April 17, 2012

@author:Isabel Restrepo

Python bindings for pcl related processes
"""

from vpcl_register import py_vpcl, dbvalue; 
import os

#********************************************************************
#  Compute Descriptor for Local Shape Features
#********************************************************************
def compute_descriptor(file_in, file_out, radius, feature_type, jobs=8, point_type="PointNormal"):
  py_vpcl.init_process("vpcl_compute_descriptor_process");
  py_vpcl.set_input_string(0,file_in);
  py_vpcl.set_input_string(1,point_type);
  py_vpcl.set_input_string(2,file_out);
  py_vpcl.set_input_string(3,feature_type);
  py_vpcl.set_input_double(4,radius);
  py_vpcl.set_input_int(5, jobs);
  py_vpcl.run_process();
  filters = dbvalue(id, type);
  return filters;

#********************************************************************
#  Compute Spin Image Descriptor
#********************************************************************
def compute_spin_image(file_in, file_out, radius, width=8, sup_angle=0.5, num_neighbors=8, radial=False, angular=False):
  py_vpcl.init_process("vpcl_compute_spin_image_process");
  py_vpcl.set_input_string(0, file_in);
  py_vpcl.set_input_double(1, radius);
  py_vpcl.set_input_int(2, width);
  py_vpcl.set_input_double(3, sup_angle);
  py_vpcl.set_input_int(4, num_neighbors);
  py_vpcl.set_input_bool(5, radial);
  py_vpcl.set_input_bool(6, angular);
  py_vpcl.set_input_string(7, file_out);
  return py_vpcl.run_process();
    
