#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_info = generate_distutils_setup(
    packages=['projector_interface'],
    package_dir={'': 'src'},
    requires=['genmsg', 'genpy', 'roslib', 'rospkg'],
    scripts=[
        'nodes/adjust_pose.py',
        'nodes/click.py',
        'nodes/find_objects.py',
        'nodes/object_circler.py',
        'nodes/object_manipulation.py',
    ],
)

setup(**package_info)
