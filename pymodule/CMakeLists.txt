# py_vpcl/CMakeLists.txt

FIND_PACKAGE ( PythonLibs )
FIND_PACKAGE (EXPAT)

IF (NOT PYTHON_FOUND AND PYTHON_LIBRARIES)
  SET(PYTHON_FOUND "YES")
ELSE (NOT PYTHON_FOUND AND PYTHON_LIBRARIES)
  SET(PYTHON_FOUND "NO")
ENDIF(NOT PYTHON_FOUND AND PYTHON_LIBRARIES)
IF (NOT PYTHON_FOUND)
  MESSAGE(STATUS "Python not found")
ENDIF (NOT PYTHON_FOUND)


#include(FindPythonInterp)
#include(FindPythonLibs  )

IF( PYTHON_FOUND AND EXPAT_FOUND)

    MESSAGE(STATUS "Found Python libraries in " ${PYTHON_INCLUDE_PATH} " and " ${PYTHON_LIBRARIES})
    MESSAGE(STATUS "Found Expat libraries in " ${EXPAT_INCLUDE_PATH} " and " ${EXPAT_LIBRARIES})


    INCLUDE_DIRECTORIES( ${PYTHON_INCLUDE_PATH} )
    INCLUDE_DIRECTORIES( ${VXL_BRL_INCLUDE_DIR} )   
    INCLUDE_DIRECTORIES( ${VXLCORE_INCLUDE_DIR} )
    INCLUDE_DIRECTORIES( ${lems_SOURCE_DIR}/contrib/dbrec_lib)
    LINK_DIRECTORIES(${PCL_LIBRARY_DIRS})

      
    SET(py_vpcl_sources
      vpcl_registry.cxx          vpcl_registry.h
    )
     
    ADD_LIBRARY(py_vpcl MODULE ${py_vpcl_sources}) 

    IF(WIN32)
      #library has to have different name depending on debug or release version.
      SET_TARGET_PROPERTIES(py_vpcl PROPERTIES OUTPUT_NAME py_vpcl DEBUG_POSTFIX _d SUFFIX .pyd)
    ENDIF(WIN32)
    SET_TARGET_PROPERTIES(py_vpcl PROPERTIES PREFIX "")

    TARGET_LINK_LIBRARIES(py_vpcl bprb brdb bpro_batch vpcl_pro ${PCL_COMMON_LIBRARY}  ${PCL_SEARCH_LIBRARY}  ${PCL_KDTREE_LIBRARY} ${PCL_FEATURES_LIBRARY} ${PCL_FILTERS_LIBRARY} ${PCL_LIBRARIES} ) 
    #TARGET_LINK_LIBRARIES(py_vpcl bprb brdb bpro_batch vpcl_pro ) 
  
    IF (PYTHON_DEBUG_LIBRARIES)
      TARGET_LINK_LIBRARIES( py_vpcl debug ${PYTHON_DEBUG_LIBRARY})
    ENDIF(PYTHON_DEBUG_LIBRARIES )

    IF(PYTHON_LIBRARIES)
      TARGET_LINK_LIBRARIES( py_vpcl optimized ${PYTHON_LIBRARY})
    ENDIF(PYTHON_LIBRARIES)

ENDIF( PYTHON_FOUND AND EXPAT_FOUND)

