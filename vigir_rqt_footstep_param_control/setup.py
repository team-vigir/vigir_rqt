from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_footstep_param_control'],
    scripts=['scripts/footstep_param_control', 'scripts/footstep_param_control_qt'],
    package_dir={'': 'src'}
)

setup(**d)
