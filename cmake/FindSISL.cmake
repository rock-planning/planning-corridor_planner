set(SISL_FOUND FALSE)

find_path(SISL_INCLUDE_DIRS "sisl/sisl.h"
    HINTS ${SISL_PREFIX}/include ${CMAKE_INSTALL_PREFIX}/include)
find_library(SISL_LIBRARIES
    NAMES libsisl.so libsisl_opt.so
    HINTS ${SISL_PREFIX}/lib ${CMAKE_INSTALL_PREFIX}/lib)

if (SISL_INCLUDE_DIRS AND SISL_LIBRARIES)
    set(SISL_FOUND TRUE)
endif()

IF (SISL_FOUND)
    IF (NOT SISL_FIND_QUIETLY)
	MESSAGE(STATUS "Found the SISL library")
    ENDIF(NOT SISL_FIND_QUIETLY)
ELSE (SISL_FOUND)
    IF (SISL_FIND_REQUIRED)
	MESSAGE(FATAL_ERROR "Please install the SISL library")
    ENDIF(SISL_FIND_REQUIRED)
ENDIF(SISL_FOUND)

