cmake_minimum_required(VERSION 2.8.3)
project(rxtools)

find_package(catkin REQUIRED COMPONENTS
  rosconsole roscpp roslib rostime xmlrpcpp wxPython_swig_interface)
catkin_package()

catkin_python_setup()

find_package(Boost REQUIRED COMPONENTS thread regex signals)
find_package(PythonLibs REQUIRED)
find_package(wxWidgets REQUIRED)
include(${wxWidgets_USE_FILE})

include_directories(include src ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS} ${PYTHON_INCLUDE_PATH} ${wxWidgets_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})

# Find the wx-enhanced swig executable
find_program(swigwx_BIN "swig" PATHS ${CMAKE_INSTALL_PREFIX}/bin)
if(${swigwx_BIN} STREQUAL "swigwx_BIN-NOTFOUND")
  message(FATAL_ERROR "wx-enhanced swig binary not found.  Aborting...")
endif()

add_definitions(-DSWIG_TYPE_TABLE=_wxPython_table -DWXP_USE_THREAD=1)

# shared library (for C++)
add_library(${PROJECT_NAME}
  src/rxtools/topic_display.cpp
  src/rxtools/topic_display_generated.cpp
  src/rxtools/topic_display_dialog.cpp
  src/rxtools/rosout_generated.cpp 
  src/rxtools/rosout_panel.cpp
  src/rxtools/rosout_filter.cpp
  src/rxtools/rosout_text_filter.cpp
  src/rxtools/rosout_text_filter_control.cpp
  src/rxtools/rosout_severity_filter.cpp
  src/rxtools/rosout_severity_filter_control.cpp
  src/rxtools/rosout_list_control.cpp
  src/rxtools/rosout_setup_dialog.cpp
  src/rxtools/logger_level_panel.cpp
  src/rxtools/init_roscpp.cpp)
add_dependencies(${PROJECT_NAME} rosgraph_msgs_gencpp)
target_link_libraries(${PROJECT_NAME} ${wxWidgets_LIBRARIES} ${Boost_LIBRARIES} ${catkin_LIBRARIES})

# Find the cpp-flags for wx which are used in CFG_EXTRAS
find_program(WX_CONFIG_BIN wx-config)
if(WX_CONFIG_BIN-NOTFOUND)
    message(FATAL_ERROR "wx-config not found!  Aborting...")
endif()
execute_process(COMMAND ${WX_CONFIG_BIN} --cppflags
  OUTPUT_VARIABLE WX_CONFIG_CPPFLAGS)
# strip newline from the end
string(REGEX REPLACE "\r?\n$" "" WX_CONFIG_CPPFLAGS "${WX_CONFIG_CPPFLAGS}")

# Add a custom command for generating the swig output files
set(SWIG_FLAGS -c++ -Wall -python -modern -new_repr -keyword -fastdispatch -fvirtual -fastinit -fastunpack ${WX_CONFIG_CPPFLAGS} -DSWIG_TYPE_TABLE=_wxPython_table -DWXP_USE_THREAD=1)

set(SWIG_INTERFACE_FILE ${PROJECT_SOURCE_DIR}/src/rxtools/rxtools.i)
set(SWIG_OUTPUT_PYTHON_FILE ${PROJECT_BINARY_DIR}/rxtoolscpp.py)

set(SWIG_OUTPUT_CPP_FILE ${PROJECT_BINARY_DIR}/rosout_panel_swig_generated.cpp)
set(SWIG_COMMAND ${swigwx_BIN} ${SWIG_FLAGS} ${wxPython_swig_interface_SWIG_FLAGS} -o ${SWIG_OUTPUT_CPP_FILE} -outdir ${PROJECT_BINARY_DIR} -module rxtoolscpp ${SWIG_INTERFACE_FILE})

set_source_files_properties(${SWIG_OUTPUT_CPP_FILE} PROPERTIES GENERATED true)
set_source_files_properties(${SWIG_OUTPUT_PYTHON_FILE} PROPERTIES GENERATED true)
add_custom_command(OUTPUT
  ${SWIG_OUTPUT_CPP_FILE} 
  ${SWIG_OUTPUT_PYTHON_FILE}
  COMMAND
  ${SWIG_COMMAND}
  DEPENDS
  ${SWIG_INTERFACE_FILE}
  src/rxtools/rosout_generated.h
  src/rxtools/rosout_panel.h
  src/rxtools/init_roscpp.h)

# We add src/wx_rosout to the include list so that
# ogre_tools_swig_generated.cpp can find its headers.  This is part of a
# workaround for a deficiency in CMake 2.4's handling of generated sources.
include_directories(${PROJECT_SOURCE_DIR}/src/rxtools)
# shared library (with SWIG)
add_library(python_${PROJECT_NAME} SHARED ${SWIG_OUTPUT_CPP_FILE})
# swig python needs a shared library named _<modulename>.[so|dll|...]
# this renames the output file to conform to that by prepending
# an underscore in place of the "lib" prefix.
# If on Darwin, force the suffix so ".so", because the MacPorts
# version of Python won't find _foo.dylib for 'import _foo'
set_target_properties(python_${PROJECT_NAME}
  PROPERTIES OUTPUT_NAME ${PROJECT_NAME}cpp
  PREFIX "_")
if(APPLE)
  set_target_properties(python_${PROJECT_NAME}
    PROPERTIES SUFFIX ".so")
endif(APPLE)

target_link_libraries(python_${PROJECT_NAME} ${PROJECT_NAME} ${PYTHON_LIBRARIES})

#rxconsole binary
add_executable(rxconsole src/rxconsole/rxconsole.cpp)
target_link_libraries(rxconsole ${PROJECT_NAME} ${catkin_LIBRARIES})

#rxloggerlevel binary
add_executable(rxloggerlevel src/rxloggerlevel/rxloggerlevel.cpp)
target_link_libraries(rxloggerlevel ${PROJECT_NAME} ${catkin_LIBRARIES})

install(PROGRAMS scripts/rxplot
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(TARGETS rxloggerlevel rxconsole
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(TARGETS rxloggerlevel rxconsole
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY icons
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE)
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY src/rxtools/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/src/rxtools
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)
install(TARGETS python_${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
install(FILES ${PROJECT_BINARY_DIR}/rxtoolscpp.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION})
