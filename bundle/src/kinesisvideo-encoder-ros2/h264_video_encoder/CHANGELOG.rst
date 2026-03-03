^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package h264_video_encoder
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

2.0.0 (2019-09-06)
------------------
* Bumping version to 2.0.0 and upgrading package.xml format to 3 (`#13 <https://github.com/aws-robotics/kinesisvideo-encoder-ros2/issues/13>`_)
  * Bumping version to 2.0.0 and upgrading package.xml format to 3
  * Add missing dependencies and tweak description
* Remove image_transport_plugins, message_generation dependencies (`#10 <https://github.com/aws-robotics/kinesisvideo-encoder-ros2/issues/10>`_)
  * Remove image_transport_plugins dependency
  * Remove dependency on message_generation
* Use undeclared node parameters
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
* Remove message_runtime from package.xml
* Add launch files and parameters (`#2 <https://github.com/aws-robotics/kinesisvideo-encoder-ros2/issues/2>`_)
  * Add launch files and parameters
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
  * Updated passing in param file
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
* Add kinesis_video_msgs to satisfy build dependency (`#4 <https://github.com/aws-robotics/kinesisvideo-encoder-ros2/issues/4>`_)
  * Add kinesis_video_msgs to satisfy build dependency
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
  * Removing launch dir for build
* Initial source code migration (`#1 <https://github.com/aws-robotics/kinesisvideo-encoder-ros2/issues/1>`_)
  * Initial source code migration
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
  * Remove unused files, update README
  * Add migrated tests
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
  * Remove unnecessary print statements
  Signed-off-by: Ryan Newell <ryanewel@amazon.com>
* Contributors: AAlon, Ryan Newell, ryanewel
