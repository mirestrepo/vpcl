# This is /pybindings. Contains all python processes/byndings to underlying c++ code

include_directories( ${VXL_BRL_INCLUDE_DIR}/bpro )
link_directories(${PCL_LIBRARY_DIRS})

include_directories( ${vpcl_SOURCE_DIR} )

set(vpcl_pro_sources
    vpcl_register.h      vpcl_register.cxx
    vpcl_processes.h
    )

aux_source_directory(processes vpcl_pro_sources)

add_library(vpcl_pro ${vpcl_pro_sources})

link_directories(${PCL_LIBRARY_DIRS})
target_link_libraries(vpcl_pro vpcl_util vpcl_reg ${PCL_LIBRARIES} ${PCL_SEARCH_LIBRARY}  ${PCL_KDTREE_LIBRARY})


