#          Copyright Maria Isabel Restrepo 2012
# Distributed under the Boost Software License, Version 1.0.
#    (See accompanying file LICENSE_1_0.txt or copy at
#          http://www.boost.org/LICENSE_1_0.txt)

cmake_minimum_required(VERSION 2.8)

project(vpcl)

find_package(VXL REQUIRED)
find_package(PCL 1.6 REQUIRED)

set(RUNTIME_OUTPUT_DIRECTORY  ${CMAKE_BINARY_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH    ${CMAKE_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH       ${CMAKE_BINARY_DIR}/lib)
set(DOCUMENTATION_OUTPUT_PATH ${CMAKE_BINARY_DIR}/doc)

set(EXECUTABLE_INSTALL_PATH    ${CMAKE_INSTALL_PREFIX}/${PREFIX}/bin/)
set(INCLUDE_INSTALL_PATH       ${CMAKE_INSTALL_PREFIX}/${PREFIX}/include/)
set(LIBRARY_INSTALL_PATH       ${CMAKE_INSTALL_PREFIX}/${PREFIX}/lib${LIB_SUFFIX}/${CMAKE_PROJECT_NAME}/)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake ${CMAKE_MODULE_PATH})

if(VXL_FOUND AND PCL_FOUND)
  # include the vxl stuff
  set(VXL_PROVIDE_STANDARD_OPTIONS 1)
  include(${VXL_CMAKE_DIR}/UseVXL.cmake)

  include_directories(${CMAKE_CURRENT_SOURCE_DIR})
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  include_directories(SYSTEM ${VXL_BRL_INCLUDE_DIR})

  
  # include the pcl stuff
  add_definitions(${PCL_DEFINITIONS})
  include_directories(${PCL_INCLUDE_DIRS})
  add_definitions(${PCL_DEFINITIONS})

  
  add_subdirectory(rply)
  add_subdirectory(pymodule)
  add_subdirectory(pybindings)
  add_subdirectory(util)

endif(VXL_FOUND AND PCL_FOUND)

