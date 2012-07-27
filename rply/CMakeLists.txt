# This is /rply/CMakeLists.txt
# library for reading and writing .PLY file

SET(rply_sources
  rply.h
  rply.c
  )
 
ADD_LIBRARY(rply ${rply_sources})

#install the .h and libs
INSTALL_TARGETS(/lib rply)
INSTALL(FILES ${rply_sources} DESTINATION /include/rply/)