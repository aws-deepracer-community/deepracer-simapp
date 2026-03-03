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
"""
This python script will help you install all the python 3.12+ dependencies to the right place.
So if you want to add a dependency for python 3.12+, add it to the install_requires list.
"""
from setuptools import setup, find_packages
 
# Package meta-data.
NAME = 'sagemaker_rl_agent'
REQUIRES_PYTHON = '>=3.12.0'
 
setup(
    name=NAME,
    version='0.0.1',
    packages=find_packages('../..', include=['markov.*']),
    package_dir={'': '../..'},
    package_data={'': ['../../markov/**']},
    python_requires=REQUIRES_PYTHON,
    install_requires=[
        'annoy==1.17.3',
        'pillow==12.1.1',
        'matplotlib==3.10.6',
        'numpy==1.26.4',
        'pandas==2.3.1',
        'pygame==2.6.1',
        'PyOpenGL==3.1.9',
        'scipy==1.13.1',
        'scikit-image==0.25.2',
        'boto3==1.38.38',
        'minio==7.2.15',
        'kubernetes==33.1.0',
        'bokeh==3.7.3',
        'pyyaml==6.0.3',
        'rospkg==1.6.0',
        'shapely==2.0.2',
        'h5py==3.14.0',
        'protobuf==6.33.5',
        'tensorflow==2.20.0',
        'opencv-python==4.9.0.80',
        'python-dateutil==2.8.2',
        'pytest==8.3.4',
        'docutils==0.21.2',
        'cryptography==46.0.5',
        'jsonschema==4.24.0',
        'pytz==2025.2',
        'tokenize-rt==6.2.0',
        'redis==8.0.2',
        'importlib-metadata==6.8.0'
    ],
    entry_points = {
        'console_scripts': [
            'run_rollout_rl_agent=markov.rollout_worker:main',
            'run_local_rl_agent=envs.local_worker:main'
        ],
    }
)