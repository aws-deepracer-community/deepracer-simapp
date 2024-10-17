if(NOT KVSSDK_EXTERNAL_INCLUDE_DIR)
  set(KVSSDK_EXTERNAL_INCLUDE_DIR ${CMAKE_BINARY_DIR}/external/include)
endif()

if(NOT KVSSDK_EXTERNAL_LIB_DIR)
  set(KVSSDK_EXTERNAL_LIB_DIR ${CMAKE_BINARY_DIR}/external/lib)
endif()

find_path(KVSSDK_PRODUCER_INCLUDE_DIR NAMES KinesisVideoProducer.h HINTS ${KVSSDK_EXTERNAL_INCLUDE_DIR}/kinesis-video-producer)
if(NOT KVSSDK_PRODUCER_INCLUDE_DIR)
  # Headers were not installed in the expected location. Try to find & install them.
  find_path(KVSSDK_PRODUCER_INCLUDE_DIR NAMES KinesisVideoProducer.h HINTS ${CMAKE_BINARY_DIR}/external/kinesis-video-producer/src)
  if(KVSSDK_PRODUCER_INCLUDE_DIR)
    file(MAKE_DIRECTORY ${KVSSDK_EXTERNAL_INCLUDE_DIR})
    file(MAKE_DIRECTORY ${KVSSDK_EXTERNAL_LIB_DIR})
    file(GLOB KINESIS_VIDEO_PRODUCER_HEADERS "${KVSSDK_PRODUCER_INCLUDE_DIR}/*.h")
    file(COPY ${KINESIS_VIDEO_PRODUCER_HEADERS} DESTINATION ${KVSSDK_EXTERNAL_INCLUDE_DIR}/kinesis-video-producer)
    set(_additional_include_dirs
      ${KVSSDK_PRODUCER_INCLUDE_DIR}/../opensource/jsoncpp/json
      ${KVSSDK_PRODUCER_INCLUDE_DIR}/../../kinesis-video-pic/src/client/include/*
      ${KVSSDK_PRODUCER_INCLUDE_DIR}/../../kinesis-video-pic/src/common/include/*
      ${KVSSDK_PRODUCER_INCLUDE_DIR}/../../kinesis-video-pic/src/utils/include/*
      ${KVSSDK_PRODUCER_INCLUDE_DIR}/../../kinesis-video-pic/src/mkvgen/include/*
      ${KVSSDK_PRODUCER_INCLUDE_DIR}/../../kinesis-video-pic/src/view/include/*
      ${KVSSDK_PRODUCER_INCLUDE_DIR}/../../kinesis-video-pic/src/heap/include/*)
    foreach(_additional_include_dir ${_additional_include_dirs})
      file(GLOB _ADDITIONAL_HEADER_FILES "${_additional_include_dir}")
      file(COPY ${_ADDITIONAL_HEADER_FILES} DESTINATION ${KVSSDK_EXTERNAL_INCLUDE_DIR}/)
    endforeach()
  endif()
endif()

if(KVSSDK_PRODUCER_INCLUDE_DIR)
  set(KVSSDK_INCLUDE_DIR ${KVSSDK_EXTERNAL_INCLUDE_DIR})
  set(KVSSDK_INCLUDE_DIRS ${KVSSDK_INCLUDE_DIR})
  if(NOT KVSSDK_LIBRARY)
    find_library(KVSSDK_LIBRARY NAMES producer libproducer HINTS ${KVSSDK_EXTERNAL_LIB_DIR})
    # Libraries were not installed in the expected location. Try to find & install them.
    if(NOT KVSSDK_LIBRARY)
      file(GLOB PRODUCER_LIB "${CMAKE_BINARY_DIR}/external/kinesis-video-native-build/downloads/local/lib/libproducer*")
      file(COPY ${PRODUCER_LIB} DESTINATION ${KVSSDK_EXTERNAL_LIB_DIR}/)
    endif()
    find_library(KVSSDK_LIBRARY NAMES producer libproducer HINTS ${KVSSDK_EXTERNAL_LIB_DIR})
    if(EXISTS "${KVSSDK_LIBRARY}")
      set(KVSSDK_LIBRARIES ${KVSSDK_LIBRARY})
      set(KVSSDK_FOUND true)
    endif()
  endif()
endif()
