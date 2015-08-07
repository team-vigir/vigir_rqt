cmake_minimum_required(VERSION 2.8.3)
project(vigir_rqt_bdi_pelvis_pose)

find_package(catkin REQUIRED COMPONENTS)

catkin_package(
    INCLUDE_DIRS src
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS)

catkin_python_setup()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

#add install rules for scripts.
catkin_install_python(PROGRAMS scripts/pelvis_pose scripts/pelvis_pose_qt
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
  
install(FILES plugin.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)


