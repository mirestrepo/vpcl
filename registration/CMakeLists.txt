# this is vpcl/registration


include_directories( util/ )

set(sources
    vpcl_corrs_util.h       vpcl_corrs_util.cxx
    vpcl_transform_util.h   vpcl_transform_util.cxx
    vpcl_transformation_estimation_svd_srt.h
    registration.h
   )
      
add_library(vpcl_reg ${sources})
link_directories(${PCL_LIBRARY_DIRS})

target_link_libraries( vpcl_reg vpcl_util vgl_algo vpgl ${PCL_COMMON_LIBRARY}  ${PCL_SEARCH_LIBRARY}  ${PCL_KDTREE_LIBRARY} ${PCL_FEATURES_LIBRARY} ${PCL_FILTERS_LIBRARY}   pcl_kdtree pcl_registration)
    
    
############################################################################
#Executables
############################################################################
add_executable(reg_test reg_test.cxx)
target_link_libraries(reg_test vpcl_reg vul pcl_visualization)
############################################################################
add_executable(visualize visualize.cxx)
target_link_libraries(visualize vpcl_reg pcl_visualization)
############################################################################
add_executable(transform_cameras_rand transform_cameras_rand.cxx)
target_link_libraries(transform_cameras_rand vpcl_reg vul vnl bxml )
############################################################################
add_executable(run_registration_ia run_registration_ia.cxx)
target_link_libraries(run_registration_ia vpcl_reg pcl_visualization )
############################################################################
add_executable(run_registration_svd run_registration_svd.cxx)
target_link_libraries(run_registration_svd vpcl_reg pcl_visualization )

