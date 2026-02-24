# Copyright 2019 Amazon.com, Inc. or its affiliates. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

macro(find_catkin)
  # Temporarily set CATKIN_INSTALL_INTO_PREFIX_ROOT to FALSE so we can call find_package(catkin)
  # without modifying the install space.
  if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT)
    set(CATKIN_INSTALL_INTO_PREFIX_ROOT_COPY ${CATKIN_INSTALL_INTO_PREFIX_ROOT})
  endif()
  set(CATKIN_INSTALL_INTO_PREFIX_ROOT FALSE)
  cmake_policy(SET CMP0048 NEW)
  find_package(catkin QUIET)
  if(DEFINED CATKIN_INSTALL_INTO_PREFIX_ROOT_COPY)
    set(CATKIN_INSTALL_INTO_PREFIX_ROOT ${CATKIN_INSTALL_INTO_PREFIX_ROOT_COPY})
  endif()
endmacro()

# macro(find_common_test_packages)
#   find_catkin()
#   find_package(ament_cmake_gtest QUIET)
#   if(catkin_FOUND)
#     catkin_destinations()
#   elseif(ament_cmake_gtest_FOUND)
#     find_package(ament_cmake_gmock REQUIRED)
#   else()
#     message(WARNING "Could not find catkin or ament!")
#   endif()

#   if(DEFINED GMOCK_LIBRARIES)
#     set(GMOCK_LIBRARY ${GMOCK_LIBRARIES})
#   else()
#     set(GMOCK_LIBRARY gmock)
#   endif()
# endmacro()

macro(add_common_gtest target)
  if(catkin_FOUND)
    message(STATUS "Building tests using catkin")
    set(GTEST_LIBRARIES "") # hack so that linking against libgmock doesn't also link against libgtest
    catkin_add_gmock("${target}" ${ARGN})
  elseif(ament_cmake_gtest_FOUND)
    message(STATUS "Building tests using ament")
    ament_add_gmock("${target}" ${ARGN})
  else()
    message(WARNING "Not building tests as neither catkin nor ament were found.")
  endif()
endmacro()
