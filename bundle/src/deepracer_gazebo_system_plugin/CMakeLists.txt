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
cmake_policy(SET CMP0054 NEW)
project(deepracer_gazebo_system_plugin)

find_package(catkin REQUIRED COMPONENTS
             gazebo_dev
             roslib
             roscpp
             geometry_msgs
             std_srvs
             std_msgs
             gazebo_msgs
             deepracer_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

catkin_package(
    LIBRARIES
        deepracer_gazebo_system_plugin

    CATKIN_DEPENDS
        roslib
        roscpp
        gazebo_ros
        geometry_msgs
        std_srvs
        std_msgs
        gazebo_msgs
        deepracer_msgs)

include_directories(
  include
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS})

link_directories(${catkin_LIBRARY_DIRS})

set(cxx_flags)
foreach (item ${GAZEBO_CFLAGS})
  set(cxx_flags "${cxx_flags} ${item}")
endforeach ()

set(ld_flags)
foreach (item ${GAZEBO_LDFLAGS})
  set(ld_flags "${ld_flags} ${item}")
endforeach ()

## Plugins
add_library(deepracer_gazebo_system_plugin src/deepracer_gazebo_system_plugin.cpp)
add_dependencies(deepracer_gazebo_system_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(deepracer_gazebo_system_plugin PROPERTIES LINK_FLAGS "${ld_flags}")
set_target_properties(deepracer_gazebo_system_plugin PROPERTIES COMPILE_FLAGS "${cxx_flags}")
target_link_libraries(deepracer_gazebo_system_plugin ${catkin_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS deepracer_gazebo_system_plugin
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
