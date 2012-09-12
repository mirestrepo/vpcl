# Source code related pcl_features library


include_directories( rply/ )

set(sources
    vpcl_features_util.h       vpcl_features_util.cxx
    vpcl_io_util.h             vpcl_io_util.cxx
   )
      
add_library(vpcl_util ${sources})
link_directories(${PCL_LIBRARY_DIRS}, ${Boost_LIBRARY_DIRS})

target_link_libraries( vpcl_util  ${PCL_COMMON_LIBRARY}  ${PCL_SEARCH_LIBRARY}  ${PCL_KDTREE_LIBRARY} ${PCL_FEATURES_LIBRARY} ${PCL_FILTERS_LIBRARY}   pcl_kdtree pcl_io rply ${Boost_SYSTEM_LIBRARY})
    
  





