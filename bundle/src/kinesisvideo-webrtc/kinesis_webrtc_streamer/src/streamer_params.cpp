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
#include "kinesis_webrtc_streamer/streamer_params.h"

#include <aws/core/client/AWSError.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <ros/ros.h>

#include "read_option.h"

namespace Aws {
namespace Kinesis {

void ReadStreamerParams(const std::shared_ptr<Aws::Client::ParameterReaderInterface> param_reader,
                        std::vector<StreamerParams> & params)
{
  int stream_count;
  ReadOption<int>(
    param_reader,
    {ros::this_node::getNamespace() + kWebRtcStreamerNamespacePrefix},
    kStreamCountKey,
    kStreamCountDefault,
    stream_count
  );

  for (int i = 0; i < stream_count; ++i) {
    std::string stream_namespace = kStreamNamespacePrefix + std::to_string(i);
    
    std::vector<std::string> parameter_namespaces{
      ros::this_node::getNamespace() + kWebRtcStreamerNamespacePrefix,
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

    ReadOption<int>(
      param_reader,
      parameter_namespaces,
      kVideoFormatTypeKey,
      static_cast<const int>(kVideoFormatTypeDefault),
      reinterpret_cast<int &>(params.back().video_format_type_)
    );
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
