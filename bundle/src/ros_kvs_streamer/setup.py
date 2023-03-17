from setuptools import find_packages
from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_name = 'ros_kvs_streamer'


PACKAGE_IMPORT = generate_distutils_setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(where='.', exclude='test'),
    package_dir={'': '.'},
    install_requires=[
        'setuptools',
        'pytest-flake8==1.0.7',
        'pytest-pep257==0.0.5',
        'pytest-timeout==1.4.2'
    ]
)

setup(**PACKAGE_IMPORT)
