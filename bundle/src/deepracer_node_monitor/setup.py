#################################################################################
#   Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.          #
#                                                                               #
#   Licensed under the Apache License, Version 2.0 (the "License").             #
#   You may not use this file except in compliance with the License.            #
#   You may obtain a copy of the License at                                     #
#                                                                               #
#       http://www.apache.org/licenses/LICENSE-2.0                              #
#                                                                               #
#   Unless required by applicable law or agreed to in writing, software         #
#   distributed under the License is distributed on an "AS IS" BASIS,           #
#   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.    #
#   See the License for the specific language governing permissions and         #
#   limitations under the License.                                              #
#################################################################################
import os
from setuptools import find_packages
from distutils.core import setup


def read(fname):
    """
    Args:
        fname (str): dir path to read
    """
    with open(os.path.join(os.path.dirname(__file__), fname)) as f:
        return f.read()


def read_version():
    return read("VERSION").strip()


package_name = 'deepracer_node_monitor'

# Declare minimal set for installation
required_packages = [
    "setuptools"
]

test_required_packages = [
    "flake8>=3.5,<4.0.0",
    "pytest-flake8==1.0.7",
    "pytest-pep257==0.0.5",
    "pytest-timeout==1.4.2",
]

setup_args = {
    "name": package_name,
    "version": read_version(),
    "packages": find_packages(where=".", exclude="test"),
    "package_dir": {"": "."},
    "description": "Open source library to communicate with remote DeepRacer environment.",
    "long_description": read("README.md"),
    "long_description_content_type": 'text/markdown',
    "author": "Amazon Web Services",
    "url": "https://github.com/aws-deepracer/deepracer-node-monitor-observer",
    "license": "Apache License 2.0",
    "keywords": "ML RL Amazon AWS AI DeepRacer",
    "classifiers": [
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Natural Language :: English",
        "License :: OSI Approved :: Apache Software License",
        "Programming Language :: Python",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
    ],
    "install_requires": required_packages,
    "tests_require": test_required_packages,
}

setup(**setup_args)
