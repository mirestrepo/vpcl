//:
// \file
// \author Isabel Restrepo
// \date 2/19/13


#include <vul/vul_arg.h>
#include <vul/vul_file.h>
#include <vul/vul_file_iterator.h>

#include <vpgl/vpgl_perspective_camera.h>

#include <iostream>
#include <string>
#include <fstream>
#include <math.h>


// the input camera is in the coordinate system of pts0. The output camera
// is the camera the coordinate system of pts 0 mapped to the
// coordinate system of pts1, that is,
//
//  x1 =  K[R0|t0](Hs Hs^-1) X1, where Hs is the similarity transform.
//
// Thus, the similarity transform is applied to the camera as,
// (s = scale)
//                        _     _  _      _
//                       |s 0 0 0||        |
//  K[R' | t'] = K[R0|t0]|0 s 0 0||  Rs  ts|
//                       |0 0 s 0||        |
//                       |0 0 0 1|| 0 0 0 1|
//                        -      - -      -
// It follows that R' = R0*Rs and t' = t0/s + R0*ts
static vpgl_perspective_camera<double>
transform_camera(vpgl_perspective_camera<double> const& cam,
                 vgl_rotation_3d<double> const& Rs,
                 vnl_vector_fixed<double, 3> const& ts,
                 const double scale)
{
  vnl_matrix_fixed<double,3,3> Rms = Rs.as_matrix();
  //Get input camera components
  //note, the homogeneous calibration matrix is unaffected by the scale
  vpgl_calibration_matrix<double> K = cam.get_calibration();
  vnl_matrix_fixed<double, 3, 3> R = cam.get_rotation().as_matrix();
  vgl_vector_3d<double> tv = cam.get_translation();
  vnl_vector_fixed<double, 3> t(tv.x(), tv.y(), tv.z());
  //compose rotations
  vnl_matrix_fixed<double, 3, 3> Rt = R*Rms;
  vgl_rotation_3d<double> Rtr(Rt);
  //compute new translation
  vnl_vector_fixed<double, 3> tt = (1.0/scale)*t + R*ts;
  vgl_vector_3d<double> ttg(tt[0], tt[1], tt[2]);
  //construct transformed camera
  vpgl_perspective_camera<double> camt(K, Rtr, ttg);
  return camt;
}




// this executable finds applies a transformation to a sequence of cameras and computes some statistics for such sequence
int main(int argc, char** argv)
{
  using namespace std;
  //Get Inputs
  vul_arg<string> geo_tform   ("-geo_tform", "Geo Transformation",  "");
  vul_arg<string> stats_file ("-stats_file", "File to save cam stats", "");
  
  vul_arg<string> input_cam_dir ("-in_cam_dir","directory to get cams","");
  vul_arg<string> output_cam_dir ("-out_cam_dir","directory to store cams", "");
  
 
  vul_arg_parse(argc, argv);
  
  // verify input camera dir
  if (!vul_file::is_directory(input_cam_dir()))
  {
    cout<<"Input Camera directory does not exist"<<endl;
    return -1;
  }
  
  // verify output camera dir
  if (!vul_file::is_directory(output_cam_dir()))
  {
    cout<<"Output Camera directory does not exist"<<endl;
    return -1;
  }
  
  //if transfomation is given, then transform the cameras
  if (!vul_file::exists(geo_tform()))
  {
    cout<<"Transform file doesn't exist"<<endl;
    return -1;
  }
  
  
  
  double scale=0.0;
  double x,y,z,r;
  vnl_vector_fixed<double, 3> t;
  
  vcl_ifstream tform_ifs;
  tform_ifs.open(geo_tform().c_str());
  tform_ifs >> scale;
  tform_ifs >> x;
  tform_ifs >> y;
  tform_ifs >> z;
  tform_ifs >> r;
  
  vgl_rotation_3d<double> R(vnl_quaternion<double>(x,y,z,r));

  tform_ifs >> t;
  
  //Transformation is assumed to transfor the points, therefore the cameras are applied the inverse transformation
  double s_inv = (1.0/scale);
  vgl_rotation_3d<double> R_inv = R.transpose();
  vnl_vector_fixed<double,3> t_inv =  R.transpose()*(scale*t);

  
  double avg_z = 0.0;
  int cam_count = 0;
  double avg_angle = 0.0;
  
  //transform the cameras
  string in_dir = input_cam_dir() + "/*.txt";
  for (vul_file_iterator fn = in_dir.c_str(); fn; ++fn, ++cam_count) {
    vcl_string f = fn();
    ifstream is;
    is.open(f.c_str());
    vpgl_perspective_camera<double> cam;
    is >> cam;
    is.close();
    vcl_string fname = vul_file::strip_directory(f.c_str());
    vcl_cout << fname << '\n';
    vpgl_perspective_camera<double> tcam = transform_camera(cam, R_inv, t_inv, s_inv);
    avg_z+=tcam.get_camera_center().z();
    avg_angle += acos(-1*tcam.principal_axis().z());
    
    string out_dir = output_cam_dir() + "/";
    vcl_string out_file = out_dir + fname;
    ofstream os(out_file.c_str());
    
    os << tcam;
    os.close();
  }
  
  //write stats file
  ofstream stats_ofs;
  stats_ofs.open(stats_file().c_str());
  if (!stats_ofs.is_open())
    return -1;
  
  stats_ofs << "ncams: " <<cam_count << "\navg_z: " << avg_z/cam_count << "\navg_angle: " << (180.0/vnl_math::pi)*avg_angle/cam_count << endl;
    
  
  
  return 0;
  
}