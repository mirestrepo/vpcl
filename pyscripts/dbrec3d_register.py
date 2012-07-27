#############################################################################
# Register Script to ensure dbrec3d_batch processes are only registered once
#
# to use this python binding, be sure to add: 
#  <lems_src>/contrib/dbrec_lib/dbrec3d/pyscripts/ 
# to your PYTHONPATH environment variable.  
#############################################################################
import dbrec3d_batch;

dbrec3d_batch.not_verbose();
dbrec3d_batch.register_processes();
dbrec3d_batch.register_datatypes();

#class used for python/c++ pointers in database
class dbvalue:
  def __init__(self, index, type):
    self.id = index    # unsigned integer
    self.type = type   # string

def remove_data(id):
  dbrec3d_batch.remove_data(id)

def get_output_float(id):
  fval = dbrec3d_batch.get_output_float(id)
  return fval

def get_output_unsigned(id):
  uval = dbrec3d_batch.get_output_unsigned(id)
  return uval

