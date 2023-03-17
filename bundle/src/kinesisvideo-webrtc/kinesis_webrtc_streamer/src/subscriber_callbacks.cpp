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
#include "kinesis_webrtc_streamer/subscriber_callbacks.h"

#include <chrono>

#include <aws/core/utils/logging/LogMacros.h>
#include <kinesis_video_msgs/KinesisVideoFrame.h>
#include <kinesis_webrtc_manager/kinesis_webrtc_manager.h>
#include <std_msgs/String.h>

namespace Aws {
namespace Kinesis {

void KinesisVideoFrameTransportCallback(
  const KinesisWebRtcManagerInterface & webrtc_manager,
  const std::string & signaling_channel_name,
  const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & msg)
{

  kinesis_video_msgs::KinesisVideoFrame frame_msg;

  frame_msg.frame_data = msg->frame_data;
  frame_msg.codec_private_data = msg->codec_private_data;
  frame_msg.index = msg->index;
  frame_msg.presentation_ts = msg->presentation_ts;
  frame_msg.flags = msg->flags;

  // prepending encoding data into the frame data if frame is a keyframe
  if (frame_msg.flags == 1) {
    frame_msg.frame_data.insert(
      frame_msg.frame_data.begin(), 
      frame_msg.codec_private_data.begin(),
      frame_msg.codec_private_data.end()
    );
  }
  Frame frame;
  frame.size = frame_msg.frame_data.size();
  frame.frameData = static_cast<PBYTE>(frame_msg.frame_data.data());
  frame.index = frame_msg.index;

  UINT64 generated_timestamp = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::system_clock::now().time_since_epoch()).count() / DEFAULT_TIME_UNIT_IN_NANOS;
  frame.presentationTs = frame_msg.presentation_ts ? frame_msg.presentation_ts : generated_timestamp;
  frame.flags = static_cast<FRAME_FLAGS>(frame_msg.flags);

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
  const std::string & message)
{
  KinesisWebRtcManagerStatus status = webrtc_manager.SendDataMessage(signaling_channel_name, false, reinterpret_cast<PBYTE>(const_cast<char *>(message.c_str())), message.length());
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
