"""
Created on April 17, 2012

@author:Isabel Restrepo

Python bindings for pcl registration processes
"""

from vpcl_register import py_vpcl, dbvalue; 
import os

    
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
  scale         = kwargs.get('scale', 1.0);

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
  py_vpcl.set_input_float(10, scale);
  return py_vpcl.run_process();


#********************************************************************
#  Perform ICP on initially aligned clouds
#********************************************************************
def register_icp(**kwargs):
  srcFname      = kwargs.get('srcFname');
  tgtFname      = kwargs.get('tgtFname');
  outCloud      = kwargs.get('outCloud');
  tformFname    = kwargs.get('tformFname');
  maxCorrDist   = kwargs.get('maxCorrDist');
  epsTrans      = kwargs.get('epsTrans', 1e-16);
  epsRot        = kwargs.get('epsRot', 1e-16);
  numIter       = kwargs.get('numIter', 200);
  maxAngDist    = kwargs.get('maxAngDist', 90);
  rejectNormals = kwargs.get('rejectNormals', False);
  useLM         = kwargs.get('useLM', False);

 
  py_vpcl.init_process("vpcl_register_icp_process");
  py_vpcl.set_input_string(0, srcFname);
  py_vpcl.set_input_string(1, tgtFname);
  py_vpcl.set_input_string(2, outCloud);
  py_vpcl.set_input_string(3, tformFname);
  py_vpcl.set_input_double(4, maxCorrDist);
  py_vpcl.set_input_double(5, epsTrans);
  py_vpcl.set_input_double(6, epsRot);
  py_vpcl.set_input_int(7, numIter);
  py_vpcl.set_input_double(8, maxAngDist);
  py_vpcl.set_input_bool(9, rejectNormals);
  py_vpcl.set_input_bool(10, useLM);
    
  return py_vpcl.run_process();
