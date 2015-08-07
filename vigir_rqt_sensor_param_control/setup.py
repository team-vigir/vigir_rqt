from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_sensor_param_control'],
    scripts=['scripts/sensor_param_control'],
    package_dir={'': 'src'}
)

setup(**d)
