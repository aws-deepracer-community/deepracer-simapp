^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aws_ros2_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.1 (2020-03-31)
------------------
* Bump package version to 1.0.1 (`#10 <https://github.com/aws-robotics/utils-ros2/issues/10>`_)
* Fix linting issues found by clang-tidy 6.0 (`#9 <https://github.com/aws-robotics/utils-ros2/issues/9>`_)
  * clang-tidy fixes
  * clang-tidy linting issues fixed manually
* Guard test targets with if(BUILD_TESTING)
* Update README.md and package.xml meta tags (`#3 <https://github.com/aws-robotics/utils-ros2/issues/3>`_)
  * Update README.md and package.xml meta tags
  * Add disclaimer about lack of apt availability
* Declare parameters in unit tests to conform to Dashing.
* Merge pull request `#2 <https://github.com/aws-robotics/utils-ros2/issues/2>`_ from aws-robotics/ci
  Add .travis.yml, .rosinstall.master, and remove aws_cpp_sdk_core obso…
* Add .travis.yml, .rosinstall.master, and remove aws_cpp_sdk_core obsolete dependency.
* Implement common interfaces of aws_common for ROS2
  * initial commit of aws_ros2_common package implementation
  * Update to use non-legacy ParameterReader API
* Contributors: AAlon, Avishay Alon, M. M, Miaofei Mei, Ryan Newell
