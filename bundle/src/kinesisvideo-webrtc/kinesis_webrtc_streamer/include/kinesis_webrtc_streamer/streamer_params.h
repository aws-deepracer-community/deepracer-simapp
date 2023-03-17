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
#include <memory>
#include <vector>
#include <string>

#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <kinesis_webrtc_manager/common.h>

namespace Aws {
namespace Kinesis {

/**
 * Structure that holds the parameters read in from deepracer_webrtc_config.yaml.
 * There is 1 structure instance for each unique stream.
 */
struct StreamerParams {
  std::string signaling_channel_name_;
  std::string video_subscription_topic_;
  std::string data_subscription_topic_;
  std::string publish_topic_;
  VideoFormatType video_format_type_;
};

/**
 * These keys are Node-wide and only read once.
 * They match the parameter names in deepracer_webrtc_config.yaml.
 */
const std::string kSpinnerThreadCountKey = "spinner_thread_count";
const std::string kStreamCountKey = "stream_count";

/**
 * These keys are Stream-specific and read once per stream.
 * They match the parameter names in deepracer_webrtc_config.yaml
 */
const std::string kSignalingChannelNameKey = "signaling_channel_name";
const std::string kVideoSubscriptionTopicKey = "video_subscription_topic";
const std::string kDataSubscriptionTopicKey = "data_subscription_topic";
const std::string kPublishTopicKey = "publish_topic";
const std::string kVideoFormatTypeKey = "video_format_type";

/**
 * These prefixes denote the namespace under which the parameter names exist.
 */
const std::string kWebRtcStreamerNamespacePrefix = "/kinesis_webrtc_streamer/webrtc_streamer_configuration";
const std::string kStreamNamespacePrefix = "stream";

/**
 * These default values are used if the configuration parameters are left blank or cannot be found.
 * kSpinnerThreadCountDefault: The default number of threads you wish to have running for the kinesis_webrtc_streamer
 * kStreamCountDefault: The default number of signaling channels we wish to open
 */
constexpr int kSpinnerThreadCountDefault = 1;
constexpr int kStreamCountDefault = 1;

const std::string kSignalingChannelNameDefault = "";
const std::string kVideoSubscriptionTopicDefault = "";
const std::string kDataSubscriptionTopicDefault = "";
const std::string kPublishTopicDefault = "";
const VideoFormatType kVideoFormatTypeDefault = VideoFormatType::H264;

/**
 * Reads in the parameters specified in deepracer_webrtc_config.yaml.
 * Initializes StreamerParams structures with the values read in 
 * and stores them in the vector passed in.
 * @param param_reader
 * @param[out] params 
 */
void ReadStreamerParams(const std::shared_ptr<Aws::Client::ParameterReaderInterface> param_reader,
                        std::vector<StreamerParams> & params);

/**
 * Reads in the AWS SDK client configuration
 * and stores it in the ClientConfiguration object passed in.
 * @param param_reader
 * @param[out] aws_config
 */
void ReadAwsConfig(const std::shared_ptr<Aws::Client::ParameterReaderInterface> param_reader,
                   Aws::Client::ClientConfiguration & aws_config);

} // namespace Kinesis
} // namespace Aws
