set (ROS_PACKAGE_NAME sandia_hand_description)

set (files
  manifest.xml
)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/ros/${ROS_PACKAGE_NAME}/)
add_subdirectory(launch)
add_subdirectory(meshes)
add_subdirectory(robots)
add_subdirectory(urdf)

