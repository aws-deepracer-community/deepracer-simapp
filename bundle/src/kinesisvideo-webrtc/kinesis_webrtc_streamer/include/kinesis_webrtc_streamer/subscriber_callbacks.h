/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once
#include <string>
#include <memory>

#include <kinesis_video_msgs/msg/kinesis_video_frame.hpp>
#include <kinesis_webrtc_manager/kinesis_webrtc_manager.h>
#include <std_msgs/msg/string.hpp>

namespace Aws {
namespace Kinesis {

/**
 * ROS-specific callback that handles a kinesis_video_msgs::msg::KinesisVideoFrame input and uses the
 * stream manager to perform a PutFrame operation.
 * @param stream_manager
 * @param frame_msg
 */
void KinesisVideoFrameTransportCallback(
  const KinesisWebRtcManagerInterface & webrtc_manager,
  const std::string & signaling_channel_name,
  const std::shared_ptr<kinesis_video_msgs::msg::KinesisVideoFrame> frame_msg
);

/**
 * ROS-specific callback that handles a std_msgs::msg::String input and uses the
 * webrtc manager to perform a SendDataMessage operation on the specified signaling channel name.
 * @param webrtc_manager
 * @param signaling_channel_name
 * @param message
 */
void StringTransportCallback(
  const KinesisWebRtcManagerInterface & webrtc_manager,
  const std::string & signaling_channel_name,
  const std::shared_ptr<std_msgs::msg::String> message
);

} // namespace Kinesis
} // namespace Aws
