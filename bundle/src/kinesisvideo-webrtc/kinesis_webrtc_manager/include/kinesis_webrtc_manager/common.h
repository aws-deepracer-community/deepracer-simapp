/*
 * Copyright 2020 Amazon.com, Inc. or its affiliates. All Rights Reserved.
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
#include <string>

#include <kinesis_webrtc_manager/kinesis_webrtc_utils.h>

namespace Aws {
namespace Kinesis {

enum class KinesisWebRtcManagerStatus {
  SUCCESS = 0,
  ERROR_BASE = 0x1000,
  BASE64DECODE_FAILED,                                /* 0x1001 */
  MALLOC_FAILED,                                      /* 0x1002 */
  INVALID_INPUT,                                      /* 0x1003 */
  PUTFRAME_SIGNALING_CHANNEL_NOT_FOUND,               /* 0x1004 */
  PUTFRAME_MISSING_KVS_WEBRTC_CONFIGURATION,          /* 0x1005 */
  PUTFRAME_NO_VIEWER,                                 /* 0x1006 */
  PUTFRAME_FAILED,                                    /* 0x1007 */
  PUTFRAME_STREAMING_SESSION_BEING_UPDATED,           /* 0x1008 */
  WEBRTC_NOT_INITIALIZED,                             /* 0x1009 */
  WEBRTC_ALREADY_INITIALIZED,                         /* 0x100A */
  EMPTY_SIGNALING_CHANNEL_NAME,                       /* 0x100B */
  SIGNALING_CHANNEL_ALREADY_EXISTS,                   /* 0x100C */
  INVALID_VIDEO_FORMAT_TYPE,                          /* 0X100D */
  CREATEKVSWEBRTCCONFIGURATION_FAILED,                /* 0x100E */
  INITKVSWEBRTC_FAILED,                               /* 0x100F */
  CREATESIGNALINGCLIENTSYNC_FAILED,                   /* 0x1010 */
  SIGNALINGCLIENTCONNECTSYNC_FAILED,                  /* 0x1011 */
  FREESIGNALINGCLIENT_FAILED,                         /* 0x1012 */
  FREEKVSWEBRTCCONFIGURATION_FAILED,                  /* 0x1013 */
  SENDDATAMESSAGE_SIGNALING_CHANNEL_NOT_FOUND,        /* 0x1014 */
  SENDDATAMESSAGE_MISSING_KVS_WEBRTC_CONFIGURATION,   /* 0x1015 */
  SENDDATAMESSAGE_NO_VIEWER,                          /* 0x1016 */
  SENDDATAMESSAGE_FAILED,                             /* 0x1017 */
  SENDDATAMESSAGE_STREAMING_SESSION_BEING_UPDATED,    /* 0x1018 */
  SESSIONCLEANUPWAIT_FAILED,                          /* 0x1019 */
};

inline bool KinesisWebRtcManagerStatusSucceeded(const KinesisWebRtcManagerStatus & status)
{
  return status == KinesisWebRtcManagerStatus::SUCCESS;
}

inline bool KinesisWebRtcManagerStatusFailed(const KinesisWebRtcManagerStatus & status)
{
  return !KinesisWebRtcManagerStatusSucceeded(status);
}

enum class VideoFormatType {
  H264 = 0,
  VP8 = 1
};

inline bool IsValidVideoFormatType(const VideoFormatType video_format_type)
{
  return video_format_type == VideoFormatType::H264 || video_format_type == VideoFormatType::VP8;
}

struct WebRtcStreamInfo {
  const UINT64 custom_data_;
  const std::string signaling_channel_name_;
  const VideoFormatType video_format_type_;
  const RtcOnMessage on_data_channel_message_callback_;
};

}  // namespace Kinesis
}  // namespace Aws
