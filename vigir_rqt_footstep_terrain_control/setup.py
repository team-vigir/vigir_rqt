from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_footstep_terrain_control'],
    scripts=['scripts/footstep_terrain_control'],
    package_dir={'': 'src'}
)

setup(**d)
