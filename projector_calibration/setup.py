#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_info = generate_distutils_setup(
    packages=['projector_calibration'],
    package_dir={'': 'src'},
    requires=['genmsg', 'genpy', 'roslib', 'rospkg'],
    scripts=['nodes/calibrate.py'],
)

setup(**package_info)
