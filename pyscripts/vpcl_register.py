#############################################################################
# Register Script to ensure py_vpcl processes are only registered once
#
# to use this python binding, be sure to add: 
#  <vpcl_src>/pyscripts/ 
# to your PYTHONPATH environment variable.  
#############################################################################
import py_vpcl;

py_vpcl.not_verbose();
py_vpcl.register_processes();
py_vpcl.register_datatypes();

#class used for python/c++ pointers in database
class dbvalue:
  def __init__(self, index, type):
    self.id = index    # unsigned integer
    self.type = type   # string

def remove_data(id):
  py_vpcl.remove_data(id)

def get_output_float(id):
  fval = py_vpcl.get_output_float(id)
  return fval

def get_output_unsigned(id):
  uval = py_vpcl.get_output_unsigned(id)
  return uval

