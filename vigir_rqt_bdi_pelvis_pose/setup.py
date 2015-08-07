from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['vigir_rqt_bdi_pelvis_pose'],
    scripts=['scripts/pelvis_pose', 'scripts/pelvis_pose_qt'],
    package_dir={'': 'src'}
)

setup(**d)
