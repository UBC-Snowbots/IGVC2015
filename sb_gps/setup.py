#!/usr/bin/env python
# DO NOT USE
# python setup.py install

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
     packages=['PYTHON_PACKAGE_NAME'],
     package_dir={'': 'src'})
setup(**d)
