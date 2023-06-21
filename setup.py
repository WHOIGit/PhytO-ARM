## ! DO NOT MANUALLY INVOKE THIS FILE, USE CATKIN INSTEAD

from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

setup_args = generate_distutils_setup(
    packages=['triton_image_classifier'],
    package_dir={'': 'src'},
    install_requires='tritonclient[all]',
)

setup(**setup_args)
