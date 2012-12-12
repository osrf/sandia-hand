set (files
  sandia_hand.launch
  sandia_hand_empty_world.launch
  upload.launch
  visualize.launch)
install(FILES ${files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/ros/${ROS_PACKAGE_NAME}/launch/)
