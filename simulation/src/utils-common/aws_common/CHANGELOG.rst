^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package aws_common
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2019-02-25)
------------------
* Add ROS2 dependencies to package.xml (`#21 <https://github.com/aws-robotics/utils-common/issues/21>`_)
  * Update package.xml to depend on ament_cmake_gtest and ament_cmake_gmock if building for ROS2.
* Disallow use of non-ParameterPath objects in ParameterReaderInterface (`#19 <https://github.com/aws-robotics/utils-common/issues/19>`_)
* Remove legacy portions of the ParameterReader API (`#18 <https://github.com/aws-robotics/utils-common/issues/18>`_)
  * remove lunar travis builds
  * remove legacy portions of the ParameterReader API
* Update ParameterReader API to support ROS1/ROS2 (`#17 <https://github.com/aws-robotics/utils-common/issues/17>`_)
  * cleanup CMakeFiles
  * refactor using the new ParameterReader API
  * clean up design of ParameterPath object
* Fix tests not running & optimize build time (`#13 <https://github.com/aws-robotics/utils-common/issues/13>`_)
* Merge pull request `#9 <https://github.com/aws-robotics/utils-common/issues/9>`_ from xabxx/master
  fixed throttling manager unit test bug
* Update local variable name to class member name
  throttled_function_call_count -> throttled_function_call_count\_
* Improve test coverage
* fixed throttling manager unit test bug
* Contributors: AAlon, Abby Xu, M. M, Ross Desmond, hortala
