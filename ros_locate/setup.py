from setuptools import setup 
from catkin_pkg.python_setup import generate_distutils_setup

setup_args= generate_distutils_setup(
	packages = ['ros_locate'],
	packe_dir = {'':'src'},
	install_requires = ['torch==1.4.0']
)

setup(**setup_args)
