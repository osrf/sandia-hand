set (model_basenames
  sandia_hand_left
  sandia_hand_right
  sandia_hand_both)

set (xacro_files
  sandia_hand_left.urdf.xacro
  sandia_hand_right.urdf.xacro
  sandia_hand_both.urdf.xacro)
install(xacro_files ${xacro_files} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}/ros/${ROS_PACKAGE_NAME}/robots/)

set (urdf_files
  sandia_hand_left.urdf
  sandia_hand_right.urdf
  sandia_hand_both.urdf)

foreach(model_basename ${model_basenames})
  #get_filename_component(basepath ${it} PATH)
  #get_filename_component(basename ${it} NAME_WE)
  add_custom_command(
    OUTPUT ${model_basename}.urdf
    COMMAND rosrun xacro xacro.py ${CMAKE_SOURCE_DIR}/ros/${ROS_PACKAGE_NAME}/robots/${model_basename}.urdf.xacro > ${model_basename}.urdf 
    DEPENDS ${model_basename}.urdf.xacro)
endforeach()
add_custom_target(plz_work ALL DEPENDS ${urdf_files})

#&& mkdir -p ../gazebo/${basename} && cp ${it} ../gazebo/${basename}/${it}
