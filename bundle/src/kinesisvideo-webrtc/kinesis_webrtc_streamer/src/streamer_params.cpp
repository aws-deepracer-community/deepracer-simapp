/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "kinesis_webrtc_streamer/streamer_params.h"

#include <aws/core/client/AWSError.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <rclcpp/rclcpp.hpp>

#include "read_option.h"

namespace Aws {
namespace Kinesis {

void ReadStreamerParams(const std::shared_ptr<Aws::Client::ParameterReaderInterface> param_reader,
                        std::vector<StreamerParams> & params)
{
  int stream_count;
  ReadOption<int>(
    param_reader,
    {kWebRtcStreamerNamespacePrefix},
    kStreamCountKey,
    kStreamCountDefault,
    stream_count
  );

  for (int i = 0; i < stream_count; ++i) {
    std::string stream_namespace = kStreamNamespacePrefix + std::to_string(i);
    
    std::vector<std::string> parameter_namespaces{
      kWebRtcStreamerNamespacePrefix,
      stream_namespace
    };

    params.push_back(StreamerParams());

    ReadOption<std::string>(
      param_reader,
      parameter_namespaces,
      kSignalingChannelNameKey,
      kSignalingChannelNameDefault,
      params.back().signaling_channel_name_
    );
    
    ReadOption<std::string>(
      param_reader,
      parameter_namespaces,
      kVideoSubscriptionTopicKey,
      kVideoSubscriptionTopicDefault,
      params.back().video_subscription_topic_
    );

    ReadOption<std::string>(
      param_reader,
      parameter_namespaces,
      kDataSubscriptionTopicKey,
      kDataSubscriptionTopicDefault,
      params.back().data_subscription_topic_
    );

    ReadOption<std::string>(
      param_reader,
      parameter_namespaces,
      kPublishTopicKey,
      kPublishTopicDefault,
      params.back().publish_topic_
    );

    int video_format_type;
    ReadOption<int>(
      param_reader,
      parameter_namespaces,
      kVideoFormatTypeKey,
      static_cast<int>(kVideoFormatTypeDefault),
      video_format_type
    );
    params.back().video_format_type_ = static_cast<VideoFormatType>(video_format_type);
  }
}

void ReadAwsConfig(const std::shared_ptr<Aws::Client::ParameterReaderInterface> param_reader,
                   Aws::Client::ClientConfiguration & aws_config)
{
  Aws::Client::ClientConfigurationProvider client_config_provider(param_reader);
  aws_config = client_config_provider.GetClientConfiguration();
  setenv("AWS_DEFAULT_REGION", aws_config.region.c_str(), 0);
  AWS_LOGSTREAM_INFO(__func__, "region is set to: " << aws_config.region);
  AWS_LOGSTREAM_INFO(__func__, "connect_timeout_ms is set to: " << aws_config.connectTimeoutMs);
  AWS_LOGSTREAM_INFO(__func__, "request_timeout_ms is set to: " << aws_config.requestTimeoutMs);
}

} // namespace Kinesis
} // namespace Aws
