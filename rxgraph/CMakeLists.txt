cmake_minimum_required(VERSION 2.8.3)
project(rxgraph)
find_package(catkin REQUIRED)
catkin_package()

find_package(wxWidgets REQUIRED)

catkin_python_setup()

install(PROGRAMS scripts/rxgraph
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
