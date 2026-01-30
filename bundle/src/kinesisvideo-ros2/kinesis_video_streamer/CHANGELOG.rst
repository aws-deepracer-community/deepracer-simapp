^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package kinesis_video_streamer
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

3.1.0 (2019-11-01)
------------------
* Bump version number to 3.1.0
* Add ability to set AWS Region and stream name via launch parameters (`#20 <https://github.com/aws-robotics/kinesisvideo-ros2/issues/20>`_)
  * add ability to override aws_region and stream_name via launch parameters
  * add ability to override rekognition_data_stream via launch parameters
  * address feedback in the pull request
  * conditionally override rekognition_data_stream, so that sample_config.yaml does not need to be modified
* Contributors: M. M, ryanewel

3.0.0 (2019-09-06)
------------------
* Delete CHANGELOG.rst
* Add changelogs
* Bumping version to 3.0.0
* Remove exports (`#15 <https://github.com/aws-robotics/kinesisvideo-ros2/issues/15>`_)
  * Remove exports from CMakeLists.txt of the streamer node
  * Remove commented out exports
* Delete BUILD.md
* Tests should verify subscription through get_topic_names_and_types API
* Increase test stability by waiting for callbacks longer and publishing less messages in kinesis_video_streamer_test.cpp
* Adding basic launch script (`#8 <https://github.com/aws-robotics/kinesisvideo-ros2/issues/8>`_)
  * Adding basic launch script
  * Fix typo/leftover in kinesis_video_streamer.launch.py
  * Set log4cplus_config path override appropriately.
* Remove install directive for launch directory as it doesn't exist yet
* Allow undeclared parameters, move spin() out of Streamer and add tests
* Adding subscriber_callbacks_test.cpp
* Restructure StreamerNode, main and add tests (`#5 <https://github.com/aws-robotics/kinesisvideo-ros2/issues/5>`_)
  * Restructure Streamer as a standalone class; move main function to main.cpp
  * Add streamer node tests after restructuring.
  * Remove unneeded code from streamer test and add an additional assertion.
* Properly set QoS settings (`#4 <https://github.com/aws-robotics/kinesisvideo-ros2/issues/4>`_)
  * Increase queue size in tests to match the amount of messages being published.
  * Use QoS settings best suited to streaming
* Pulling changes from ROS1: CI setup, bug fix in subscriber_callbacks.cpp, minor test stability fixes. (`#2 <https://github.com/aws-robotics/kinesisvideo-ros2/issues/2>`_)
  * Pulling changes from ROS1: CI setup, bug fix in subscriber_callbacks.cpp, minor test stability fixes.
  * Empty commit to trigger CI.
* Add packages kinesis_video_streamer, kinesis_video_msgs (`#1 <https://github.com/aws-robotics/kinesisvideo-ros2/issues/1>`_)
* Contributors: AAlon, Avishay Alon
