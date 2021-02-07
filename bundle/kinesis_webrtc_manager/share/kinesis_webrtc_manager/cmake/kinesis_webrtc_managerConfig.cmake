# Compute paths
set(kinesis_webrtc_manager_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../../include")

if(NOT TARGET kinesis_webrtc_manager)
  include("${CMAKE_CURRENT_LIST_DIR}/kinesis_webrtc_manager-targets.cmake")
endif()

set(kinesis_webrtc_manager_LIB_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib")
set(kinesis_webrtc_manager_LIBRARIES kinesis_webrtc_manager libkvsWebrtcClient.so;libkvsWebrtcSignalingClient.so;libkvsCommonLws.a)

# Imported packages which the current package intends to export.
foreach(_imported_library libkvsWebrtcClient.so;libkvsWebrtcSignalingClient.so;libkvsCommonLws.a)
  if(NOT TARGET ${_imported_library})
    add_library(${_imported_library} UNKNOWN IMPORTED)
    set_target_properties(${_imported_library} PROPERTIES
      INTERFACE_INCLUDE_DIRECTORIES ${kinesis_webrtc_manager_INCLUDE_DIRS}
      IMPORTED_CONFIGURATIONS NOCONFIG
      IMPORTED_LOCATION_NOCONFIG ${kinesis_webrtc_manager_LIB_DIR}/${_imported_library}
    )
  endif()
endforeach()

# where the .pc pkgconfig files are installed
set(kinesis_webrtc_manager_PKGCONFIG_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../lib/pkgconfig")
