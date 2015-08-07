from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['trapezoidal_trajectory_widget'],
    scripts=['scripts/trapezoidal_trajectory_widget'],
    package_dir={'': 'src'}
)

setup(**d)
