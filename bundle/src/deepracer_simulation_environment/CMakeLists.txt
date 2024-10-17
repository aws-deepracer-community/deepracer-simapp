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

cmake_minimum_required(VERSION 2.8.3)
project(deepracer_simulation_environment)

find_package(catkin REQUIRED COMPONENTS
             gazebo_ros
             std_msgs
             cv_bridge
             sensor_msgs
             image_transport
             message_generation)

catkin_python_setup()

add_message_files(FILES
    AgentRewardData.msg
)

add_service_files(FILES
    TopCamDataSrv.srv
    VideoMetricsSrv.srv
    VirtualEventVideoEditSrv.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
                  std_msgs
                  sensor_msgs)


catkin_package(CATKIN_DEPENDS
               message_runtime
               std_msgs
               sensor_msgs
               image_transport
               cv_bridge)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

#############
## Install ##
#############

install(PROGRAMS
    scripts/run_local_rl_agent.sh
    scripts/run_rollout_rl_agent.sh
    scripts/run_evaluation_rl_agent.sh
    scripts/run_virtual_event_rl_agent.sh
    scripts/only_sleep.sh
    scripts/car_node.py
    scripts/car_control_webrtc_msg_node.py
    scripts/visualization_node.py
    scripts/download_params_and_roslaunch_agent.py
    scripts/mp4_saving/agents_video_editor.py
    scripts/mp4_saving/virtual_event_video_editor.py
    scripts/test_fixture_nodes/camera_topic_node.py
    tests/pytest_runner.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY ../../meshes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)
install(DIRECTORY ../../models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)
install(DIRECTORY ../../urdf/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdf
)
install(DIRECTORY ../../worlds/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/worlds
)
install(DIRECTORY ../../routes/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/routes
)
install(DIRECTORY fonts/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/fonts
)
install(DIRECTORY track_iconography/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/track_iconography
)
install(DIRECTORY ../../track_iconography/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/track_iconography
)
install(DIRECTORY tests/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)
install(DIRECTORY tests/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/tests
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest(tests/test_agents_video_editor.launch)
endif()