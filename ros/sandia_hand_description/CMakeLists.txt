set (ROS_PACKAGE_NAME sandia_hand_description)

set (files
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${SANDIA_HAND_VERSION}/ros/${ROS_PACKAGE_NAME}/)
add_subdirectory(launch)
add_subdirectory(robots)
add_subdirectory(urdf)

