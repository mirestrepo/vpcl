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
    
#********************************************************************
#  Compute Rigid Transformation
#********************************************************************
def compute_rigid_transformation(src_fname, tgt_fname, src_features_fname, tgt_features_fname, tform_cloud_fname, tform_fname, feature_type):
  py_vpcl.init_process("vpcl_feature_based_rigid_transform_process");
  py_vpcl.set_input_string(0, src_fname);
  py_vpcl.set_input_string(1, tgt_fname);
  py_vpcl.set_input_string(2, src_features_fname);
  py_vpcl.set_input_string(3, tgt_features_fname);
  py_vpcl.set_input_string(4, tform_cloud_fname);
  py_vpcl.set_input_string(5, tform_fname);
  py_vpcl.set_input_string(6, feature_type);
  return py_vpcl.run_process();

#********************************************************************
#  Compute Initial Aligment using IA_SAC
#********************************************************************
def register_ia_sac(**kwargs):
  srcFname      = kwargs.get('srcFname');
  tgtFname      = kwargs.get('tgtFname');
  srcFeatures   = kwargs.get('srcFeatures');
  tgtFeatures   = kwargs.get('tgtFeatures');
  outCloud      = kwargs.get('outCloud');
  tformFname    = kwargs.get('tformFname');
  descType      = kwargs.get('descType', 'FPFH');
  minSampleDist = kwargs.get('minSampleDist');
  maxCorrDist   = kwargs.get('maxCorrDist');
  numIter       = kwargs.get('numIter', 200);

  py_vpcl.init_process("vpcl_register_ia_process");
  py_vpcl.set_input_string(0, srcFname);
  py_vpcl.set_input_string(1, tgtFname);
  py_vpcl.set_input_string(2, srcFeatures);
  py_vpcl.set_input_string(3, tgtFeatures);
  py_vpcl.set_input_string(4, outCloud);
  py_vpcl.set_input_string(5, tformFname);
  py_vpcl.set_input_string(6, descType);
  py_vpcl.set_input_double(7, minSampleDist);
  py_vpcl.set_input_double(8, maxCorrDist);
  py_vpcl.set_input_int(9, numIter);
  return py_vpcl.run_process();


#********************************************************************
#  Perform ICP on initially aligned clouds
#********************************************************************
def register_icp(**kwargs):
  srcFname      = kwargs.get('srcFname');
  tgtFname      = kwargs.get('tgtFname');
  outCloud      = kwargs.get('outCloud');
  tformFname    = kwargs.get('tformFname');
  maxCorrDist = kwargs.get('maxCorrDist');
  outlierThresh   = kwargs.get('outlierThresh');
  epsTrans   = kwargs.get('epsTrans', 1e-16);
  epsEucle   = kwargs.get('epsEucle', 1e-16);
  numIter       = kwargs.get('numIter', 200);
  
  py_vpcl.init_process("vpcl_register_icp_process");
  py_vpcl.set_input_string(0, srcFname);
  py_vpcl.set_input_string(1, tgtFname);
  py_vpcl.set_input_string(2, outCloud);
  py_vpcl.set_input_string(3, tformFname);
  py_vpcl.set_input_double(4, maxCorrDist);
  py_vpcl.set_input_double(5, outlierThresh);
  py_vpcl.set_input_double(6, epsTrans);
  py_vpcl.set_input_double(7, epsEucle);
  py_vpcl.set_input_int(8, numIter);
  return py_vpcl.run_process();