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

#include <kinesis_video_msgs/KinesisVideoFrame.h>
#include <kinesis_webrtc_manager/kinesis_webrtc_manager.h>

namespace Aws {
namespace Kinesis {

/**
 * ROS-specific callback that handles a kinesis_video_msgs::KinesisVideoFrame input and uses the
 * stream manager to perform a PutFrame operation.
 * @param stream_manager
 * @param frame_msg
 */

/*
void KinesisVideoFrameTransportCallback(
  const KinesisWebRtcManagerInterface & webrtc_manager,
  const std::string & signaling_channel_name,
  const kinesis_video_msgs::msg::KinesisVideoFrame::SharedPtr frame_msg
);
*/

void KinesisVideoFrameTransportCallback(
  const KinesisWebRtcManagerInterface & webrtc_manager,
  const std::string & signaling_channel_name,
  const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & msg
);

/**
 * ROS-specific callback that handles a std::string input and uses the
 * webrtc manager to perform a SendDataMessage operation on the specified signaling channel name.
 * @param webrtc_manager
 * @param signaling_channel_name
 * @param message
 */
void StringTransportCallback(
  const KinesisWebRtcManagerInterface & webrtc_manager,
  const std::string & signaling_channel_name,
  const std::string & message
);

} // namespace Kinesis
} // namespace Aws
