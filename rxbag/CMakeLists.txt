cmake_minimum_required(VERSION 2.8)
project(rxbag)
find_package(catkin REQUIRED)
catkin_project(rxbag)

install(PROGRAMS scripts/rxbag
  DESTINATION ${PROJECT_LIBEXEC_DESTINATION})

install(DIRECTORY icons
  DESTINATION ${PROJECT_SHARE_DESTINATION})
