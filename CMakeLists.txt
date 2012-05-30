cmake_minimum_required(VERSION 2.8)
project(rx)

find_package(catkin REQUIRED)

find_package(wxWidgets REQUIRED)
catkin_stack()

foreach(subdir
    xdot
    rxgraph
    wxPython_swig_interface
    rxtools
    rxbag
    )
  add_subdirectory(${subdir})
endforeach()

# install all manifests
install_matching_to_share(manifest.xml)

install(FILES stack.xml
        DESTINATION share/rx)

catkin_python_setup()
