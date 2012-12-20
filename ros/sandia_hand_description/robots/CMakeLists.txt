set (model_basenames
     sandia_hand_left
     sandia_hand_right
     sandia_hand_both)

foreach(model_basename ${model_basenames})
  add_custom_command(
    OUTPUT ${model_basename}.urdf
    COMMAND rosrun xacro xacro.py ${CMAKE_SOURCE_DIR}/ros/${ROS_PACKAGE_NAME}/robots/${model_basename}.urdf.xacro > ${model_basename}.urdf 
    DEPENDS ${model_basename}.urdf.xacro)
  add_custom_target(${model_basename}_urdf ALL DEPENDS ${model_basename}.urdf)
  install(FILES ${model_basename}.urdf.xacro
          DESTINATION ${ROS_PKG_INSTALL_PATH}/robots/)
  install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${model_basename}.urdf
          DESTINATION ${ROS_PKG_INSTALL_PATH}/gazebo/${model_basename})
endforeach()

