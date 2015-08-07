from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_control_mode'],
    scripts=['scripts/control_mode', 'scripts/control_mode_qt'],
    package_dir={'': 'src'}
)

setup(**d)
