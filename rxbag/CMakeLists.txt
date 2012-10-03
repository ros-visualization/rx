cmake_minimum_required(VERSION 2.8.3)
project(rxbag)
find_package(catkin REQUIRED)
catkin_project(rxbag)

install(PROGRAMS scripts/rxbag
  DESTINATION ${CATKIN_PROJECT_BIN_DESTINATION})

install(DIRECTORY icons
  DESTINATION ${CATKIN_PROJECT_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
