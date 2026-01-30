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
#include <image_transport/image_transport.hpp>
#include <kinesis_manager/kinesis_stream_manager.h>
#include "kinesis_video_msgs/msg/kinesis_video_frame.hpp"
#include "std_msgs/msg/string.hpp"

namespace Aws {
namespace Kinesis {

/**
 * ROS2-specific callback that handles a kinesis_video_msgs::KinesisVideoFrame input and uses the
 * stream manager to perform a PutFrame operation.
 * @param stream_manager
 * @param stream_name
 * @param frame_msg
 */
void KinesisVideoFrameTransportCallback(
    KinesisStreamManagerInterface & stream_manager, std::string stream_name,
    const kinesis_video_msgs::msg::KinesisVideoFrame::ConstSharedPtr frame_msg);
/**
 * This callback uses the above KinesisVideoFrameTransportCallback, then fetches and publishes the
 * analysis results from AWS Rekognition.
 * @param stream_manager
 * @param stream_name
 * @param frame_msg
 * @param publisher
 */
void RekognitionEnabledKinesisVideoFrameTransportCallback(
    KinesisStreamManagerInterface & stream_manager, std::string stream_name,
    const kinesis_video_msgs::msg::KinesisVideoFrame::ConstSharedPtr frame_msg,
    const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher);
/**
 * ROS2-specific callback that handles a sensor_msgs::Image input and uses the stream manager
 *  to perform a PutFrame operation.
 * @param stream_manager
 * @param stream_name
 * @param image
 */
void ImageTransportCallback(const KinesisStreamManagerInterface & stream_manager,
                            std::string stream_name,
                            const sensor_msgs::msg::Image::ConstSharedPtr image);

} // namespace Kinesis
} // namespace Aws