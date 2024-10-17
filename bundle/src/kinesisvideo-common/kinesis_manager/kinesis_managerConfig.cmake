# Compute paths
set(kinesis_manager_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include")

if(NOT TARGET kinesis_manager)
  include("${CMAKE_CURRENT_LIST_DIR}/kinesis_manager-targets.cmake")
endif()

set(kinesis_manager_LIBRARIES kinesis_manager)

# where the .pc pkgconfig files are installed
set(kinesis_manager_PKGCONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/pkgconfig")
