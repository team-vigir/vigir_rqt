from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_system_command'],
    scripts=['scripts/system_command'],
    package_dir={'': 'src'}
)

setup(**d)
