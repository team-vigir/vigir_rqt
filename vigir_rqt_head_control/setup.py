from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_head_control'],
    scripts=['scripts/head_control'],
    package_dir={'': 'src'}
)

setup(**d)
