cmake_minimum_required(VERSION 2.8.3)
project(deepracer_msgs)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs
  gazebo_msgs
  std_srvs
  message_generation
  )

add_service_files(DIRECTORY srv FILES
  GetLightNames.srv
  GetVisualNames.srv
  GetVisual.srv
  GetVisuals.srv
  GetLinkStates.srv
  GetModelStates.srv
  SetVisualColor.srv
  SetVisualColors.srv
  SetVisualTransparency.srv
  SetVisualTransparencies.srv
  SetVisualVisible.srv
  SetVisualVisibles.srv
  SetVisualPose.srv
  SetVisualPoses.srv
  SetVisualMesh.srv
  SetVisualMeshes.srv
  SetLinkStates.srv
  SetModelStates.srv
  )

generate_messages(DEPENDENCIES
  std_msgs
  geometry_msgs
  gazebo_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  std_msgs
  geometry_msgs
  gazebo_msgs
  std_srvs
)
