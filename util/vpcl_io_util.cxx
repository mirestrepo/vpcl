//:
// \file
// \author Isabel Restrepo
// \date 26-Jan-2012


#include "vpcl_io_util.h"


//template <class PointType>
bool vpcl_io_util::load_cloud (const string &filename, pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
  cout << "Loading: " << filename <<endl;
  
  string file_type = vul_file::extension(filename);
  if (file_type == ".pcd") {
    if (pcl::io::loadPCDFile (filename, *cloud) < 0)
      return (false);
  }
  else if (file_type == ".ply"){
    pcl::PLYReader reader;
    cout << " Reading..." << endl;
//    if(!reader.read (filename, *cloud)){
//      cout<< "PCL PLY reader failed, trying custom reader \n";
//      cloud->clear();
//      if(! normals_pcd_from_ply(filename, cloud)){
//        cout<< "Errorr: Custom PLY reader failed too\n";
//        return false;
//      }
//    }
    normals_pcd_from_ply(filename, cloud);
  }
  else {
    cout << "File type not supported: " << file_type << endl;
    return false;
  }
  
  
  cout << " Done:" <<  cloud->width * cloud->height << " point" << endl;
  cout << "Available dimensions: " << getFieldsList(*cloud).c_str () << endl;
  cout << "First point: " << cloud->points[0] << endl;
  
  return true;
}


//: The PLY reader of PCL is rather strict, so lets load the cloud on our own
bool vpcl_io_util::normals_pcd_from_ply(const string &filename, pcl::PointCloud<pcl::PointNormal>::Ptr cloud)
{
  ply_normals_reader parsed_ply;
  parsed_ply.cloud = cloud;
  
  p_ply ply = ply_open(filename.c_str(), NULL, 0, NULL);
  if (!ply) {
    cout << "File " << filename << " doesn't exist.";
    return false;
  }
  if (!ply_read_header(ply)){
    cout << "File " << filename << " doesn't have header.";
    return false;
  }
  
  // vertex
  int nvertices = ply_set_read_cb(ply, "vertex", "x", plyio_vertex_cb, (void*) (&parsed_ply), 0);
  ply_set_read_cb(ply, "vertex", "y", plyio_vertex_cb, (void*) (&parsed_ply), 1);
  ply_set_read_cb(ply, "vertex", "z", plyio_vertex_cb, (void*) (&parsed_ply), 2);
  ply_set_read_cb(ply, "vertex", "nx", plyio_vertex_cb, (void*) (&parsed_ply), 3);
  ply_set_read_cb(ply, "vertex", "ny", plyio_vertex_cb, (void*) (&parsed_ply), 4);
  ply_set_read_cb(ply, "vertex", "nz", plyio_vertex_cb, (void*) (&parsed_ply), 5);
  ply_set_read_cb(ply, "vertex", "normal_x", plyio_vertex_cb, (void*) (&parsed_ply), 3);
  ply_set_read_cb(ply, "vertex", "normal_y", plyio_vertex_cb, (void*) (&parsed_ply), 4);
  ply_set_read_cb(ply, "vertex", "normal_z", plyio_vertex_cb, (void*) (&parsed_ply), 5);

  
  // Read DATA
  ply_read(ply);

  
  // CLOSE file
  ply_close(ply);
  
  cloud=parsed_ply.cloud;
  
  return true;
}


//: Call-back function for a "vertex" element
int vpcl_io_util::plyio_vertex_cb(p_ply_argument argument)
{
  long index;
  void* temp;
  ply_get_argument_user_data(argument, &temp, &index);
  
  ply_normals_reader* parsed_ply =  (ply_normals_reader*) temp;
  
  switch (index)
  {
    case 0: // "x" coordinate
      parsed_ply->p.x = ply_get_argument_value(argument);
      break;
    case 1: // "y" coordinate
      parsed_ply->p.y = ply_get_argument_value(argument);
      break;
    case 2: // "z" coordinate
      parsed_ply->p.z= ply_get_argument_value(argument);
      break;
    case 3: // "nx" coordinate
      parsed_ply->p.normal_x =ply_get_argument_value(argument);
      break;
    case 4:// "ny" coordinate
      parsed_ply->p.normal_y = ply_get_argument_value(argument);
      break;
    case 5: // "nz" coordinate
      parsed_ply->p.normal_z = ply_get_argument_value(argument);
      parsed_ply->p.curvature = 0.0;  //dummy curvature
      // Insert into the cloud
      parsed_ply->cloud->push_back(parsed_ply->p);
      break;
    default:
      assert(!"This should not happen: index out of range");
  }
  return 1;
}


//load correspondences from file - comma separated
pcl::CorrespondencesPtr vpcl_io_util::loadCorrespondences(const string& name)
{
  pcl::CorrespondencesPtr corrs(new pcl::Correspondences);
  
  //load from file
  ifstream file(name.c_str());
  if (file.is_open())
  {
    string line;
    while ( file.good() ) {
      getline (file,line);
      vector<string> strNums = split(line);
      if(strNums.size()<3)
        continue;
      int index_query = fromString<int>(strNums[0]);
      int index_match = fromString<int>(strNums[1]);
      float distance  = fromString<float>(strNums[2]);
      pcl::Correspondence corr(index_query, index_match, distance);
      corrs->push_back(corr);
    }
    file.close();
  }
  else {
    cout << "Unable to open correspondence file "<<name<<endl;
  }
  return corrs;
}


//save correspondences to text file
void vpcl_io_util::saveCorrespondences(const string& name, pcl::CorrespondencesPtr corrs )
{
  ofstream file;
  file.open (name.c_str());
  for(int i=0; i<corrs->size(); ++i) {
    file << corrs->at(i) << '\n';
  }
  file.close();
}

//String split helper methods....
vector<string>& vpcl_io_util::split(const string &s, char delim, vector<string> &elems) {
  stringstream ss(s);
  string item;
  while(getline(ss, item, delim)) {
    elems.push_back(item);
  }
  return elems;
}

vector<string> vpcl_io_util::split(const string &s, char delim) {
  vector<string> elems;
  return split(s, delim, elems);
}
