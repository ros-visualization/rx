cmake_minimum_required(VERSION 2.8.3)
project(wxpython_swig_interface)
find_package(catkin REQUIRED)
catkin_package(CFG_EXTRAS wxpython_swig_interface-extras.cmake)

catkin_python_setup()

install(DIRECTORY src/wxpython_swig_interface
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src)
