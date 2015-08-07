from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_position_mode'],
    scripts=['scripts/position_mode', 'scripts/position_mode_qt'],
    package_dir={'': 'src'}
)

setup(**d)
