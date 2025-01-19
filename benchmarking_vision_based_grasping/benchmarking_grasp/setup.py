## ! DO NOT MANUALLY INVOKE THIS setup.py, USE CATKIN INSTEAD

from setuptools import setup
from catkin_pkg.python_setup import generate_distutils_setup

# fetch values from package.xml
setup_args = generate_distutils_setup(
    packages=['benchmarking_pipeline_module', 'grasp_transform_module'],
    package_dir={'': 'src'})

setup(**setup_args)
