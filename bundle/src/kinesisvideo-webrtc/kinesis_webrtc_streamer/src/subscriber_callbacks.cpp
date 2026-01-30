/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "kinesis_webrtc_streamer/subscriber_callbacks.h"

#include <chrono>
#include <memory>

#include <aws/core/utils/logging/LogMacros.h>
#include <kinesis_video_msgs/msg/kinesis_video_frame.hpp>
#include <kinesis_webrtc_manager/kinesis_webrtc_manager.h>
#include <std_msgs/msg/string.hpp>

namespace Aws {
namespace Kinesis {

void KinesisVideoFrameTransportCallback(
  const KinesisWebRtcManagerInterface & webrtc_manager,
  const std::string & signaling_channel_name,
  const std::shared_ptr<kinesis_video_msgs::msg::KinesisVideoFrame> frame_msg)
{
  // prepending encoding data into the frame data if frame is a keyframe
  if (frame_msg->flags == 1) {
    frame_msg->frame_data.insert(
      frame_msg->frame_data.begin(), 
      frame_msg->codec_private_data.begin(),
      frame_msg->codec_private_data.end()
    );
  }
  
  Frame frame;
  frame.size = frame_msg->frame_data.size();
  frame.frameData = static_cast<PBYTE>(frame_msg->frame_data.data());
  frame.index = frame_msg->index;

  UINT64 generated_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count() / DEFAULT_TIME_UNIT_IN_NANOS;
  frame.presentationTs = frame_msg->presentation_ts ? frame_msg->presentation_ts : generated_timestamp;
  frame.flags = static_cast<FRAME_FLAGS>(frame_msg->flags);

  KinesisWebRtcManagerStatus status = webrtc_manager.PutFrame(signaling_channel_name, &frame);
  if (status == KinesisWebRtcManagerStatus::PUTFRAME_NO_VIEWER) {
    AWS_LOGSTREAM_DEBUG(__func__, signaling_channel_name << " has no viewers");
  } else if (KinesisWebRtcManagerStatusFailed(status)) {
    AWS_LOGSTREAM_WARN(__func__, signaling_channel_name << " PutFrame failed.");
  } else {
    AWS_LOGSTREAM_DEBUG(__func__, signaling_channel_name << " PutFrame succeeded. Frame Index: " << frame.index);
  }
}

void StringTransportCallback(
  const KinesisWebRtcManagerInterface & webrtc_manager,
  const std::string & signaling_channel_name,
  const std::shared_ptr<std_msgs::msg::String> message)
{
  KinesisWebRtcManagerStatus status = webrtc_manager.SendDataMessage(
    signaling_channel_name, 
    false, 
    reinterpret_cast<PBYTE>(const_cast<char *>(message->data.c_str())), 
    message->data.length());
    
  if (status == KinesisWebRtcManagerStatus::SENDDATAMESSAGE_NO_VIEWER) {
    AWS_LOGSTREAM_DEBUG(__func__, signaling_channel_name << " has no viewers");
  } else if (KinesisWebRtcManagerStatusFailed(status)) {
    AWS_LOGSTREAM_WARN(__func__, signaling_channel_name << " SendDataMessage failed.");
  } else {
    AWS_LOGSTREAM_DEBUG(__func__, signaling_channel_name << " SendDataMessage succeeded.");
  }
}

} // namespace Kinesis
} // namespace Aws
