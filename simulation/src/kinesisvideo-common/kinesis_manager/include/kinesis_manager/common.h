/*
 * Copyright 2018 Amazon.com, Inc. or its affiliates. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License").
 * You may not use this file except in compliance with the License.
 * A copy of the License is located at
 *
 *  http://aws.amazon.com/apache2.0
 *
 * or in the "license" file accompanying this file. This file is distributed
 * on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either
 * express or implied. See the License for the specific language governing
 * permissions and limitations under the License.
 */
#pragma once
#include <sstream>

#define KINESIS_MANAGER_STATUS_SUCCEEDED(status) (!(KINESIS_MANAGER_STATUS_FAILED(status)))
#define KINESIS_MANAGER_STATUS_FAILED(status) \
  ((status)&Aws::Kinesis::KINESIS_MANAGER_STATUS_ERROR_BASE)

#ifndef INVALID_STREAM_ID
#define INVALID_STREAM_ID (-1)
#endif

namespace Aws {
namespace Kinesis {

typedef enum kinesis_manager_status_e {
  KINESIS_MANAGER_STATUS_SUCCESS = 0,
  KINESIS_MANAGER_STATUS_ERROR_BASE = 0x1000,
  KINESIS_MANAGER_STATUS_BASE64DECODE_FAILED,                         /* 0x1001 */
  KINESIS_MANAGER_STATUS_MALLOC_FAILED,                               /* 0x1002 */
  KINESIS_MANAGER_STATUS_INVALID_INPUT,                               /* 0x1003 */
  KINESIS_MANAGER_STATUS_CREATESTREAMSYNC_FAILED,                     /* 0x1004 */
  KINESIS_MANAGER_STATUS_PUTFRAME_STREAM_NOT_FOUND,                   /* 0x1005 */
  KINESIS_MANAGER_STATUS_PUTFRAME_FAILED,                             /* 0x1006 */
  KINESIS_MANAGER_STATUS_PUTMETADATA_STREAM_NOT_FOUND,                /* 0x1007 */
  KINESIS_MANAGER_STATUS_PUTMETADATA_FAILED,                          /* 0x1008 */
  KINESIS_MANAGER_STATUS_VIDEO_PRODUCER_NOT_INITIALIZED,              /* 0x1009 */
  KINESIS_MANAGER_STATUS_VIDEO_PRODUCER_ALREADY_INITIALIZED,          /* 0x100A */
  KINESIS_MANAGER_STATUS_DEFAULT_CREDENTIAL_PROVIDER_CREATION_FAILED, /* 0x100B */
  KINESIS_MANAGER_STATUS_EMPTY_STREAM_NAME,                           /* 0x100C */
  KINESIS_MANAGER_STATUS_STREAM_ALREADY_EXISTS,                       /* 0x100D */
  KINESIS_MANAGER_STATUS_SUBSCRIPTION_INSTALLATION_FAILED,            /* 0x100E */
  KINESIS_MANAGER_STATUS_SUBSCRIPTION_INSTALLER_NOT_FOUND,            /* 0x100F */
  KINESIS_MANAGER_STATUS_PROCESS_CODEC_DATA_STREAM_CONFIG_NOT_FOUND,  /* 0x1010 */
  KINESIS_MANAGER_STATUS_GET_STREAM_DEFINITION_FAILED,                /* 0x1011 */
  KINESIS_MANAGER_STATUS_NO_STREAMS_SPECIFIED,                        /* 0x1012 */
  KINESIS_MANAGER_STATUS_LIST_SHARDS_FAILED,                          /* 0x1013 */
  KINESIS_MANAGER_STATUS_LIST_SHARDS_EMPTY,                           /* 0x1014 */
  KINESIS_MANAGER_STATUS_GET_SHARD_ITERATOR_FAILED,                   /* 0x1015 */
  KINESIS_MANAGER_STATUS_GET_RECORDS_FAILED,                          /* 0x1016 */
  KINESIS_MANAGER_STATUS_GET_RECORDS_THROTTLED,                       /* 0x1017 */
} KinesisManagerStatus;

typedef int KinesisStreamInputType;
constexpr uint32_t kDefaultMessageQueueSize = 1000;
constexpr uint16_t kDefaultRecordsLimitForRekognitionResults = 50;

/**
 * The following structure represents the format in which parameters for the streamer node are
 * expected. kinesis_video/ stream_count: int stream<idx>/ subscription_topic: string
 *   subscription_queue_size: optional int, defaults to 100.
 *   topic_type: int in range of KinesisStreamInputType
 *   < StreamDefinition parameters as defined in
 * https://docs.aws.amazon.com/kinesisvideostreams/latest/dg/how-data.html#how-data-header-streamdefinition
 * >
 *  ... more streams ...
 */
const struct
{
  const char * prefix;
  const char * stream_count;
  const char * stream_namespace;
  const char * topic_name;
  const char * message_queue_size;
  const char * topic_type;
  const char * log4cplus_config;
  const char * stream_name;
  const char * rekognition_topic_name;
  const char * rekognition_data_stream;
} kStreamParameters{
  "kinesis_video",
  "stream_count",
  "stream",
  "subscription_topic",
  "subscription_queue_size", /* Overrides kDefaultMessageQueueSize */
  "topic_type",              /* Topic type value as defined in KinesisStreamInputType */
  "log4cplus_config",        /* Path to a log4cplus configuration file for use by the Kinesis Video
                                Producer SDK */
  "stream_name",
  "rekognition_topic_name", /* AWS Rekognition analysis results will be published to this topic. */
  "rekognition_data_stream" /* AWS Rekognition analysis results will be read from this stream. Must
                               be provided if topic_type is Rekognition-enabled. */
};

}  // namespace Kinesis
}  // namespace Aws