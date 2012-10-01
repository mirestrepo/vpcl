//:
// \file
// \brief Executable to apply a similarity transform to camera matrices. 
// \author Isabel Restrepo
// \date 8/1/12

#include "vpcl_transform_util.h"

#include <vnl/vnl_random.h>
#include <vnl/vnl_quaternion.h>
#include <vnl/vnl_inverse.h>
#include <vgl/algo/vgl_rotation_3d.h>
#include <vgl/vgl_box_3d.h>
#include <vpgl/vpgl_perspective_camera.h>
#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include <vul/vul_file_iterator.h>

#include <iostream>
#include<fstream>

#include <bxml/bxml_write.h>
#include <bxml/bxml_read.h>
#include <bxml/bxml_find.h>


using namespace std;

bool parse_scene_info(const string &xml_file, vnl_vector_fixed<double,3> &min, vnl_vector_fixed<double,3> &max, float &resolution)
{
  ifstream xml_ifs(xml_file.c_str());
  if(!xml_ifs.is_open()){
    cerr << "Error: could not open xml info file: " << xml_file << endl;
    return false;
  }
  bxml_document doc = bxml_read(xml_ifs);
  bxml_element root_query("bwm_info_for_boxm2");
  bxml_data_sptr root = bxml_find_by_name(doc.root_element(), root_query);
  if (!root) {
    cerr << "Error: could not parse xml root" << endl;
    return false;
  }
  
  //Parse bbox
  bxml_element query("bbox");
  bxml_data_sptr data = bxml_find_by_name(root, query);
  
  if (!data) {
    cerr << "Error: could not parse bbox tag" << endl;
    return false;
  }
  
  bxml_element* bbox_elm = dynamic_cast<bxml_element*>(data.ptr());
  double min_x, min_y, min_z, max_x, max_y, max_z = 0.0;
  bbox_elm->get_attribute("minx", min_x);
  bbox_elm->get_attribute("miny", min_y);
  bbox_elm->get_attribute("minz", min_z);
  bbox_elm->get_attribute("maxx", max_x);
  bbox_elm->get_attribute("maxy", max_y);
  bbox_elm->get_attribute("maxz", max_z);
  
  min[0] = min_x; min[1] = min_y; min[2] = min_z;
  max[0] = max_x; max[1] = max_y; max[2] = max_z;
  
  
  bxml_element res_query("resolution");
  bxml_data_sptr res_data = bxml_find_by_name(root, res_query);
  
  if (!res_data) {
    cerr << "Error: could not parse resolution tag" << endl;;
    return false;
  }
  
  bxml_element* res_elm = dynamic_cast<bxml_element*>(res_data.ptr());
  resolution =0.0f;
  res_elm->get_attribute("val", resolution);
  
  
  //close the xml doc
  xml_ifs.close();
  return true;
}

void write_scene_info(const string &xml_file, const vnl_vector_fixed<double,3> &min, const vnl_vector_fixed<double,3> &max, const float &resolution)
{
  //write bounding boxm to xml file
  bxml_document doc;
  bxml_element *root = new bxml_element("bwm_info_for_boxm2");
  doc.set_root_element(root);
  root->append_text("\n");

  bxml_element* bbox_elm = new bxml_element("bbox");
  bbox_elm->append_text("\n");
  bbox_elm->set_attribute("minx", min[0] );
  bbox_elm->set_attribute("miny", min[1]);
  bbox_elm->set_attribute("minz", min[2] );
  bbox_elm->set_attribute("maxx", max[0] );
  bbox_elm->set_attribute("maxy", max[1] );
  bbox_elm->set_attribute("maxz", max[2] );
  root->append_data(bbox_elm);
  root->append_text("\n");
  
  //write resolution to xml
  bxml_element* res_elm = new bxml_element("resolution");
  res_elm->append_text("\n");
  res_elm->set_attribute("val", resolution);
  root->append_data(res_elm);
  root->append_text("\n");
  
  //write the number of trees
  bxml_element* ntrees_elm = new bxml_element("ntrees");
  ntrees_elm->append_text("\n");
  ntrees_elm->set_attribute("ntrees_x", 48);
  ntrees_elm->set_attribute("ntrees_y", 48);
  ntrees_elm->set_attribute("ntrees_z", 48);
  root->append_data(ntrees_elm);
  root->append_text("\n");
  
  //write to disk
  ofstream xml_os(xml_file.c_str());
  bxml_write(xml_os, doc);
  xml_os.close();

}

vnl_matrix_fixed<double,4,4> compute_similarity(vgl_rotation_3d<double> const& R,
                                                vnl_vector_fixed<double, 3> const& T,
                                                const double S)
{
  //get the similarity matrix form S, R, T
  vnl_matrix_fixed<double,4,4> SM(0.0);
  for(unsigned i= 0; i<3; i++)
    SM(i,i) = S;
  SM(3,3) = 1.0;
  vnl_matrix_fixed<double,4,4> RT(0.0);
  for(unsigned i= 0; i<2; i++)
    RT(i,3) = T(i);
  RT(3,3) = 1.0;
  
  RT.update(R.as_matrix());
  
  vnl_matrix_fixed<double,4,4> Hs = SM * RT;
  
  return Hs;
}

//: Executable to apply a similarity transform to camera matrices.
//  Optionally it applies the transformation to the bbox of a boxm2 scene
int main(int argc, char *argv[])
{
  
  //Get Inputs
  vul_arg<const char *> input_dir ("-in_dir","directory to scene root - typically contains cams_krt/ and scene_info.xml","");
  vul_arg<const char *> output_dir ("-out_dir", "directory to store /cams_KRT dir and the file with the applied transformation", "");
  vul_arg<double> dt ("-dt", "the translation will be uniformly sampled from [-5*dt, 5*dt)"); 


  if(argc < 3) {
    cout<<"usage: ./transform_cameras_rand -in_cam_dir <cam_dir> -out_dir <out_dir> -dt <translation unit>"<<endl;
    return -1;
  }
  
  vul_arg_parse(argc, argv);

  // verify input camera dir
  if (!vul_file::is_directory(input_dir()))
  {
    cout<<"Input directory does not exist"<<endl;
    return -1;
  }
  
  // verify output camera dir
  if (!vul_file::is_directory(output_dir()))
  {
    cout<<"Output directory does not exist -- creating one "<<endl;
    vul_file::make_directory_path(output_dir());
  }

  double t_sup = 5.0*dt();
  
  vnl_random rnd;

  ///////////////////////////////////////////////////////////////////
  //Generate a random translation
  vnl_vector_fixed<double, 3> T(rnd.drand64(-t_sup,t_sup), rnd.drand64(-t_sup,t_sup), rnd.drand64(-t_sup,t_sup));
  ///////////////////////////////////////////////////////////////////

  ///////////////////////////////////////////////////////////////////
  //Generate a random rotation - using Shoemake's quaternion method//
  ///////////////////////////////////////////////////////////////////
  double x0 = rnd.drand64(0,1);
  double y1 = 2.0*vnl_math::pi*rnd.drand64(0,1);
  double y2 = 2*vnl_math::pi*rnd.drand64(0,1);
  
  double r1 = sqrt(1.0-x0);
  double r2 = sqrt(x0);
  
  //Compute quaternion entries
  double u0 = cos(y2)*r2;
  double u1 = sin(y1)*r1;
  double u2 = cos(y1)*r1;
  double u3 = sin(y2)*r2;
  
  //Create the rotation from the random quaternion
  vnl_quaternion<double> q(u0, u1, u2, u3);
  vgl_rotation_3d<double> R(q);
  
  ///////////////////////////////////////////////////////////////////
  //Generate a random scale                                        //
  ///////////////////////////////////////////////////////////////////
//  double S = double(rnd.lrand32(1, 1));
//  int shrink = rnd.lrand32 (0,1);
//  if(shrink)
//    S = 1.0/S;
  
  double S = 1.0;
  
  cout << "Transformation: \nScale = " << S << "\nR = " << R.as_matrix() << "\nt = " << T << '\n';

  
  ////////////////////////////////////////////////////////////////////////
  //Transform cameras with a random similarity matrix
  //  x1 =  K[R0|t0](Hs Hs^-1) X1, where Hs is the similarity transform.
  //  X1 = HsX0  or X1 = s*(Rs*X0 + ts)
  //                        _     _  _      _
  //                       |s 0 0 0||        |
  //  K[R' | t'] = K[R0|t0]|0 s 0 0||  Rs  ts|
  //                       |0 0 s 0||        |
  //                       |0 0 0 1|| 0 0 0 1|
  //                        -      - -      -
  // It follows that R' = R0*Rs and t' = t0/s + R0*ts
  ////////////////////////////////////////////////////////////////////////
  
  //transform the cameras
  string in_dir = input_dir();
  in_dir = in_dir + "/cams_krt/*.txt";
  
  string cam_out_dir = output_dir();
  cam_out_dir = cam_out_dir + "/cams_krt/";

  // verify output camera dir
  if (!vul_file::is_directory(cam_out_dir.c_str()))
  {
    cout<<"Camera directory does not exist -- creating one "<<endl;
    vul_file::make_directory_path(cam_out_dir);
  }
  
  for (vul_file_iterator fn = in_dir.c_str(); fn; ++fn) {
    string f = fn();
    ifstream is(fn(), ifstream::in);
    vpgl_perspective_camera<double> cam;
    is >> cam;
    is.close();
    string fname = vul_file::strip_directory(f.c_str());
    cout << fname << '\n';
    vpgl_perspective_camera<double> tcam =
    vpcl::transform::transform_camera(cam, R, T, S);
    string out_file = cam_out_dir + fname;
    ofstream os(out_file.c_str());
    os << tcam;
    os.close();
  }
  
  string out_file = string(output_dir()) + "/Hs.txt";
  ofstream os(out_file.c_str());
  os << S << '\n'
  << R << '\n'
  << T << '\n';
  os.close();
  
  //inverse transformation
  double S_inv = (1.0/S);
  vgl_rotation_3d<double> R_inv = R.transpose();
  vnl_vector_fixed<double,3> T_inv =  R.transpose()*(S*T);
  
  out_file = string(output_dir()) + "/Hs_inv.txt";
  ofstream os2(out_file.c_str());
  os2 << S_inv << '\n'
  << R_inv << '\n'
  << -1.0* T_inv << '\n';
  os2.close();
  
  //If at the given input directory, we find a scene_info.xml, the the transformation will be applied to bbox and resolution
  string xml_file_in = string(input_dir()) + "/scene_info.xml";
  
  if(! vul_file::exists(xml_file_in))
    return 0;
  
  vnl_vector_fixed<double,3> min(0.0);
  vnl_vector_fixed<double,3> max(0.0);
  float resolution = 0.0;
  
  if(!parse_scene_info(xml_file_in, min, max, resolution))
    return -1;
  cout << "Done parsing bbox:\nMin: " << min << "\nMax: " << max << "\nResolution: " << resolution << endl;
  
  ////////////////////////////////////////////////////////////////////////
  //Transform the bbox - transfoming the min and max points is not sufficient
  ////////////////////////////////////////////////////////////////////////

  vgl_box_3d<double> bbox(min[0], min[1], min[2], max[0], max[1], max[2]);
  
  vcl_vector<vgl_point_3d<double> > vertices = bbox.vertices();
  vgl_box_3d<double> new_bbox;
  
  for (unsigned i=0; i < vertices.size(); i++)
  {
    vnl_vector_fixed<double, 3> vert(vertices[i].x(), vertices[i].y(), vertices[i].z());
    vnl_vector_fixed<double, 3> new_vert = S_inv*(R_inv*vert - T_inv);
    new_bbox.add(vgl_point_3d<double>(new_vert[0], new_vert[1], new_vert[2]));
  }
    

  
//  vnl_vector_fixed<double,3> new_min = R.transpose()*((1.0/S)*min - T);
//  vnl_vector_fixed<double,3> new_max = R.transpose()*((1.0/S)*max - T);
  double new_res = resolution/S;
  vnl_vector_fixed<double,3> new_min(new_bbox.min_x(), new_bbox.min_y(), new_bbox.min_z());
  vnl_vector_fixed<double,3> new_max(new_bbox.max_x(), new_bbox.max_y(), new_bbox.max_z());
  
  string xml_file_out = string(output_dir()) + "/scene_info.xml";

  write_scene_info(xml_file_out, new_min, new_max, new_res);
  cout << "Done writing bbox:\nMin: " << new_min << "\nMax: " << new_max << "\nResolution: " << new_res << endl;

}