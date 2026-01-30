# Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.8)
project(deepracer_simulation_environment)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/AgentRewardData.msg"
  "srv/TopCamDataSrv.srv"
  "srv/VideoMetricsSrv.srv"
  "srv/VirtualEventVideoEditSrv.srv"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

# Install Python scripts
install(PROGRAMS
    scripts/car_control_webrtc_msg_node.py
    scripts/car_node.py
    scripts/car_control_webrtc_msg_node.py
    scripts/download_params_and_roslaunch_agent.py
    scripts/only_sleep.sh
    scripts/run_local_rl_agent.sh
    scripts/run_rollout_rl_agent.sh
    scripts/run_evaluation_rl_agent.sh
    scripts/run_virtual_event_rl_agent.sh
    scripts/visualization_node.py
    scripts/test_fixture_nodes/camera_topic_node.py
    scripts/mp4_saving/agents_video_editor.py
    scripts/mp4_saving/virtual_event_video_editor.py
    tests/pytest_runner.py
    DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY scripts/mp4_saving/
    DESTINATION lib/${PROJECT_NAME}/mp4_saving
)

# Install directories
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY params/
  DESTINATION share/${PROJECT_NAME}/params
)
install(DIRECTORY fonts/
  DESTINATION share/${PROJECT_NAME}/fonts
)
install(DIRECTORY track_iconography/
  DESTINATION share/${PROJECT_NAME}/track_iconography
)
install(DIRECTORY tests/
  DESTINATION share/${PROJECT_NAME}/tests
)

# Install external assets if they exist
if(EXISTS /opt/amazon/meshes/)
  install(DIRECTORY /opt/amazon/meshes/
    DESTINATION share/${PROJECT_NAME}/meshes
  )
endif()
if(EXISTS /opt/amazon/models/)
  install(DIRECTORY /opt/amazon/models/
    DESTINATION share/${PROJECT_NAME}/models
  )
endif()
if(EXISTS /opt/amazon/urdf/)
  install(DIRECTORY /opt/amazon/urdf/
    DESTINATION share/${PROJECT_NAME}/urdf
  )
endif()
if(EXISTS /opt/amazon/worlds/)
  install(DIRECTORY /opt/amazon/worlds/
    DESTINATION share/${PROJECT_NAME}/worlds
  )
endif()
if(EXISTS /opt/amazon/routes/)
  install(DIRECTORY /opt/amazon/routes/
    DESTINATION share/${PROJECT_NAME}/routes
  )
endif()
if(EXISTS /opt/amazon/track_iconography/)
  install(DIRECTORY /opt/amazon/track_iconography/
    DESTINATION share/${PROJECT_NAME}/track_iconography
  )
endif()

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  
  ament_lint_auto_find_test_dependencies()
  
  # Add pytest tests
  ament_add_pytest_test(test_agents_video_editor tests/mp4_saving/test_agents_video_editor.py
    TIMEOUT 60
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  )
endif()

ament_package()
