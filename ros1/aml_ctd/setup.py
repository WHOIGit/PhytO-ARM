## ! DO NOT MANUALLY INVOKE THIS FILE, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    py_modules=['amlxparser'],
    package_dir={'': 'src'})

setup(**setup_args)
