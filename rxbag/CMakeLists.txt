cmake_minimum_required(VERSION 2.8.3)
project(rxbag)
find_package(catkin REQUIRED)
catkin_package()

find_package(wxWidgets REQUIRED)

catkin_python_setup()

install(PROGRAMS scripts/rxbag
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
