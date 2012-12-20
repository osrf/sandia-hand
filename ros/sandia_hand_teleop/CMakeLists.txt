cmake_minimum_required(VERSION 2.8.0)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)
rosbuild_init()
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
set(ROS_PKG_INSTALL_SHARE_PATH ${CMAKE_INSTALL_PREFIX}/share/sandia-hand/ros/${PROJECT_NAME})

install(FILES manifest.xml 
        DESTINATION ${ROS_PKG_INSTALL_SHARE_PATH})

add_subdirectory(control_nodes)
add_subdirectory(input_nodes)
