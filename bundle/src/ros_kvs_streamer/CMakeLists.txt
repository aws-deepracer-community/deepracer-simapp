cmake_minimum_required(VERSION 3.7.2)
project(ros_kvs_streamer)

find_package(catkin REQUIRED COMPONENTS
    rospy
)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS
    rospy
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)

## Tests
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(pytest.launch)
endif()
