set (files
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/ros/${ROS_PACKAGE_NAME}/gazebo/)

