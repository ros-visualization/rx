@[if BUILDSPACE]@
# swig flags in buildspace
  set(@(PROJECT_NAME)_SWIG_FLAGS "-I@(CMAKE_CURRENT_SOURCE_DIR)/src/wxpython_swig_interface")
@[else]@
# swig flags in installspace
  set(@(PROJECT_NAME)_SWIG_FLAGS "-I@(CMAKE_INSTALL_PREFIX)/@(CATKIN_PACKAGE_SHARE_DESTINATION)/src/wxpython_swig_interface")
@[end if]@
