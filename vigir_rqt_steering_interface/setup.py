from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_steering_interface'],
    scripts=['scripts/steering_interface'],
    package_dir={'': 'src'}
)

setup(**d)
