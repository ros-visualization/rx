cmake_minimum_required(VERSION 2.8.3)
project(wxpython_swig_interface)
find_package(catkin REQUIRED)
catkin_package(CFG_EXTRAS rx_wxPython-extras.cmake)

catkin_python_setup()
