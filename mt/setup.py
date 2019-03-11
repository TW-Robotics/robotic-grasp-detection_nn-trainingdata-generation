from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

# Make packages findable with ros

# fetch values from package.xml
setup_args = generate_distutils_setup(
	packages=['ur5_control'],
	package_dir={'': 'src'},
)

setup(**setup_args)
