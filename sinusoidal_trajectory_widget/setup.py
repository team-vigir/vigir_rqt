from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['sinusoidal_trajectory_widget'],
    scripts=['scripts/sinusoidal_trajectory_widget'],
    package_dir={'': 'src'}
)

setup(**d)
