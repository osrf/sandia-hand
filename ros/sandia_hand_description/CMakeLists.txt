cmake_minimum_required(VERSION 2.8.3)
project(sandia_hand_description)

find_package(catkin REQUIRED COMPONENTS xacro)

# catkin_python_setup()

catkin_package(
  CATKIN_DEPENDS xacro
)

install(DIRECTORY
  config launch meshes urdf gazebo robots
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#what to do about isntalling robots?
#add_subdirectory(robots)










