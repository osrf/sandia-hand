set (files
  manifest.xml
)
install(FILES manifest.xml
        DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}-${SANDIA_HAND_VERSION}/ros/${ROS_PACKAGE_NAME}/gazebo/
        RENAME database.config
       )


