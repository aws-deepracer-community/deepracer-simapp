#----------------------------------------------------------------
# Generated CMake target import file.
#----------------------------------------------------------------

# Commands may need to know the format version.
set(CMAKE_IMPORT_FILE_VERSION 1)

# Import target "kinesis_webrtc_manager" for configuration ""
set_property(TARGET kinesis_webrtc_manager APPEND PROPERTY IMPORTED_CONFIGURATIONS NOCONFIG)
set_target_properties(kinesis_webrtc_manager PROPERTIES
  IMPORTED_LOCATION_NOCONFIG "${_IMPORT_PREFIX}/lib/libkinesis_webrtc_manager.so"
  IMPORTED_SONAME_NOCONFIG "libkinesis_webrtc_manager.so"
  )

list(APPEND _IMPORT_CHECK_TARGETS kinesis_webrtc_manager )
list(APPEND _IMPORT_CHECK_FILES_FOR_kinesis_webrtc_manager "${_IMPORT_PREFIX}/lib/libkinesis_webrtc_manager.so" )

# Commands beyond this point should not need to know the version.
set(CMAKE_IMPORT_FILE_VERSION)
