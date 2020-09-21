# Copyright (c) Facebook, Inc. and its affiliates.

# This source code is licensed under the MIT license found in the
# LICENSE file in the root directory of this source tree.

#!/usr/bin/env python

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['kinematics'],
    package_dir={'': 'src'},
    requires=['kdl_pybind', 'trac_ik_pybind', 'urdf_parser_py', 'numpy',
              'rospy', 'geometry_msgs', 'sensor_msgs']
)

setup(**d)
