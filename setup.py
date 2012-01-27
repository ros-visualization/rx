#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

setup(name='rx',
      version= '1.7.1',
      packages=[
          'rxbag',
          'rxbag.plugin',
          'rxbag.util',
          'rxgraph',
          'rxtools',
          'wxPython_swig_interface',
          'xdot',
                ],
      package_dir = {
          'rxbag':'rxbag/src/rxbag',
          'rxbag.plugin':'rxbag/src/rxbag/plugin',
          'rxbag.util':'rxbag/src/rxbag/util',
          'rxgraph':'rxgraph/src/rxgraph',
          'rxtools':'rxtools/src/rxtools',
          'wxPython_swig_interface':'wxPython_swig_interface/src',
          'xdot':'xdot/src/xdot',
                     },
      install_requires=['rospkg', 'roslib', 'ros_comm'],
      scripts = [
          'rxbag/scripts/rxbag',
          'rxgraph/scripts/rxgraph',
          'rxtools/scripts/rxplot',
                 ],
      author = "Maintained by Ken Conley", 
      author_email = "kwc@willowgarage.com",
      url = "http://www.ros.org/wiki/rx",
      download_url = "http://pr.willowgarage.com/downloads/rx/", 
      keywords = ["ROS"],
      classifiers = [
        "Programming Language :: Python", 
        "License :: OSI Approved :: BSD License" ],
      description = "rx Python libraries and tools",
      long_description = """\
Python libraries and toolchain for ROS GUI-related packages.
""",
      license = "BSD"
      )

