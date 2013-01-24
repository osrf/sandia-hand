cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
enable_testing()

project (sandia-hand)
include(ExternalProject)
string(REPLACE "-" "_" PROJECT_IDENT ${PROJECT_NAME})
string(TOUPPER ${PROJECT_IDENT} PROJECT_IDENT)

set (SANDIA_HAND_MAJOR_VERSION 5)
set (SANDIA_HAND_MINOR_VERSION 1)
set (SANDIA_HAND_PATCH_VERSION 7)
set (SANDIA_HAND_VERSION ${SANDIA_HAND_MAJOR_VERSION}.${SANDIA_HAND_MINOR_VERSION})
set (SANDIA_HAND_VERSION_FULL ${SANDIA_HAND_MAJOR_VERSION}.${SANDIA_HAND_MINOR_VERSION}.${SANDIA_HAND_PATCH_VERSION})
message (STATUS "${PROJECT_NAME} version ${SANDIA_HAND_VERSION_FULL}")

# Packaging configuration
set (CPACK_PACKAGE_VERSION "${SANDIA_HAND_VERSION_FULL}")
set (CPACK_PACKAGE_VERSION_MAJOR "${SANDIA_HAND_MAJOR_VERSION}")
set (CPACK_PACKAGE_VERSION_MINOR "${SANDIA_HAND_MINOR_VERSION}")
set (CPACK_PACKAGE_VERSION_PATCH "${SANDIA_HAND_PATCH_VERSION}")
list (APPEND CPACK_SOURCE_IGNORE_FILES ";/.hg/;.hgignore;.swp$;/build/")
set (CPACK_SOURCE_PACKAGE_FILE_NAME "@PROJECT_NAME@-@SANDIA_HAND_VERSION_FULL@")
include (CPack)

set (ROS_DISTRO "fuerte")
if (NOT EXISTS "$ENV{ROS_ROOT}")
  message(FATAL_ERROR "Cannot find ROS_ROOT. Ensure that ROS is installed and source the appropriate ROS configuration file (e.g. /opt/ros/${ROS_DISTRO}/setup.sh).")
endif()

set (INCLUDE_INSTALL_DIR "include/${PROJECT_NAME}-${SANDIA_HAND_VERSION}/${PROJECT_NAME}") 
set (LIB_INSTALL_DIR "lib" CACHE STRING "Installation directory for libraries (relative to CMAKE_INSTALL_PREFIX)") 
set (BIN_INSTALL_DIR "bin" CACHE STRING "Installation directory for binaries (relative to CMAKE_INSTALL_PREFIX)") 

#####################################
# Set the default build type
if (NOT CMAKE_BUILD_TYPE)
  set (CMAKE_BUILD_TYPE "RELEASE" CACHE STRING 
    "Choose the type of build, options are: debug release profile check" FORCE)
endif (NOT CMAKE_BUILD_TYPE)
string(TOUPPER ${CMAKE_BUILD_TYPE} CMAKE_BUILD_TYPE)

add_subdirectory(ros)

set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(conf_file "${CMAKE_CURRENT_SOURCE_DIR}/cmake/sandia-hand-config.cmake")
configure_file("${conf_file}.in" "${PROJECT_BINARY_DIR}/${conf_file}" @ONLY)
install(FILES "${PROJECT_BINARY_DIR}/${conf_file}" DESTINATION share/${PROJECT_NAME}-${SANDIA_HAND_VERSION}/cmake COMPONENT cmake)
