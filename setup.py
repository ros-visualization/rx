#!/usr/bin/env python

from setuptools import setup

import sys
sys.path.insert(0, 'src')

setup(name='rx',
      version= '1.7.0',
      packages=[
          'rxgraph',
          'rxtools',
          'xdot',
                ],
      package_dir = {
          'rxgraph':'rxgraph/src/rxgraph',
          'rxtools':'rxtools/src/rxtools',
          'xdot':'xdot/src/xdot',
                     },
      install_requires=['rospkg', 'roslib', 'ros_comm'],
      scripts = [
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

