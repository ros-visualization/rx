cmake_minimum_required(VERSION 2.8)
project(wxswig)
find_package(catkin)

# Find the cpp-flags for wx which are used in CFG_EXTRAS
find_program(WX_CONFIG_BIN wx-config)
if(WX_CONFIG_BIN-NOTFOUND)
    message(FATAL_ERROR "wx-config not found!  Aborting...")
endif()
execute_process(COMMAND ${WX_CONFIG_BIN} --cppflags
                OUTPUT_VARIABLE WX_CONFIG_CPPFLAGS)
# strip newline from the end
string(REGEX REPLACE "\r?\n$" "" WX_CONFIG_CPPFLAGS "${WX_CONFIG_CPPFLAGS}")

catkin_project(wxswig
  CFG_EXTRAS rx_wxswig-extras.cmake
  )

file(COPY Makefile md5.checksum swig-wrap
    DESTINATION external_make
)

add_custom_target(external_make ALL
    COMMAND make -f ${PROJECT_BINARY_DIR}/external_make/Makefile
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/external_make
)

