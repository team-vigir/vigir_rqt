from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_bias_calibration'],
    scripts=['scripts/bias_calibration'],
    package_dir={'': 'src'}
)

setup(**d)
