# Created by Manuel Blum on 2011-05-25.
# Copyright 2013 University of Freiburg.

CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

PROJECT(libgp CXX C)

# if no option is given, standard is release
IF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
   SET(CMAKE_BUILD_TYPE Release)
ENDIF(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)

# Check if usage in ros_libgp_bridge
IF(DEFINED LIBGP_LIB_PATH)
    OPTION(USAGE_IN_ROS_BRIDGE "Sets switch for usage in ros_libgp_dridge to true" ON)
ELSE()
    OPTION(USAGE_IN_ROS_BRIDGE "Sets switch for usage in ros_libgp_dridge to false" OFF)
ENDIF()

set(CMAKE_CXX_FLAGS_RELEASE "-std=c++14 -DCLSQUARE -Wall -O2 -fPIC")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++14 -DCLSQUARE -Wall -g -fPIC")

IF(USAGE_IN_ROS_BRIDGE)
    OPTION(BUILD_TESTS "Build tests" OFF)
    OPTION(BUILD_EXAMPLES "Build examples" OFF)
ELSE()
    OPTION(BUILD_TESTS "Build tests" OFF)
    OPTION(BUILD_EXAMPLES "Build examples" OFF)
ENDIF()

SET(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR} ${CMAKE_MODULE_PATH})

FIND_PACKAGE(Eigen3 3.0.1 REQUIRED)

# ----- Add include directories -----
INCLUDE_DIRECTORIES(SYSTEM ${EIGEN3_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${CMAKE_CURRENT_SOURCE_DIR}/include)

# ----- Add sources -----
INCLUDE("Sources.cmake")

# ----- Add library target -----
ADD_LIBRARY(gp ${LIBGP_SRC} ${LIBGP_INTERFACES})

# ----- Generate PKgconfig file ----
get_property(LIB64 GLOBAL PROPERTY FIND_LIBRARY_USE_LIB64_PATHS)
if ("${LIB64}" STREQUAL "TRUE")
    set(LIBSUFFIX 64)
else()
    set(LIBSUFFIX "")
endif()
set(INSTALL_LIB_DIR lib${LIBSUFFIX} CACHE PATH
    "Installation directory for libraries")
mark_as_advanced(INSTALL_LIB_DIR)
IF(USAGE_IN_ROS_BRIDGE)
    configure_file(${LIBGP_LIB_PATH}/contrib/libgp.pc.in 
	${CMAKE_BINARY_DIR}/libgp.pc @ONLY)
ELSE()
    configure_file(${CMAKE_SOURCE_DIR}/contrib/libgp.pc.in 
	${CMAKE_BINARY_DIR}/libgp.pc @ONLY)
ENDIF()

# ----- Add example targets -----
IF(BUILD_EXAMPLES)
  ADD_SUBDIRECTORY(examples)
ENDIF()



# ----- Install -----
INSTALL(FILES ${LIBGP_INTERFACES} DESTINATION include/gp)
INSTALL(TARGETS gp
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION ${INSTALL_LIB_DIR}
  ARCHIVE DESTINATION ${INSTALL_LIB_DIR}
)
INSTALL(FILES ${CMAKE_BINARY_DIR}/libgp.pc DESTINATION ${INSTALL_LIB_DIR}/pkgconfig)
