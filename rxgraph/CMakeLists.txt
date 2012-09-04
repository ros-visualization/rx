cmake_minimum_required(VERSION 2.8)
project(rxgraph)
find_package(catkin REQUIRED)
catkin_project(rxgraph)

install(PROGRAMS scripts/rxgraph
  DESTINATION ${PROJECT_LIBEXEC_DESTINATION})
