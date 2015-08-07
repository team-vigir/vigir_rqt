from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_stable_step'],
    scripts=['scripts/stable_step'],
    package_dir={'': 'src'}
)

setup(**d)
