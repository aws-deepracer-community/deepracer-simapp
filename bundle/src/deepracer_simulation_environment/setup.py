""" This ensures modules and global scripts declared therein get installed
## See http://ros.org/doc/api/catkin/html/user_guide/setup_dot_py.html
"""
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

PACKAGE_IMPORT = generate_distutils_setup(
    packages=['mp4_saving', 'test_fixture_nodes', 'mp4_saving.states'],
    package_dir={'': 'scripts'}
)

setup(**PACKAGE_IMPORT)
