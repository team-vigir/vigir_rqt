from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_no_limit_joint_control'],
    scripts=['scripts/no_limit_joint_control'],
    package_dir={'': 'src'}
)

setup(**d)
