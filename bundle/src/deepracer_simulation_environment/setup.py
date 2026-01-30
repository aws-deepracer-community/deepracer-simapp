# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

from setuptools import setup, find_packages

package_name = 'deepracer_simulation_environment'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(where='scripts'),
    package_dir={'': 'scripts'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AWS DeepRacer',
    maintainer_email='aws-deepracer@amazon.com',
    description='The AWS DeepRacer simulation environment package',
    license='Apache2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'car_node = car_node:main',
            'car_control_webrtc_msg_node = car_control_webrtc_msg_node:main',
            'visualization_node = visualization_node:main',
            'download_params_and_roslaunch_agent = download_params_and_roslaunch_agent:main',
        ],
    },
)
