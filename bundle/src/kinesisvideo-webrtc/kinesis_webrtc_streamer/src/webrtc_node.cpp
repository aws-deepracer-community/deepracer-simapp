/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include "kinesis_webrtc_streamer/webrtc_node.h"

#include <memory>

#include <std_msgs/msg/string.hpp>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros2_common/sdk_utils/ros2_node_parameter_reader.h>
#include <kinesis_video_msgs/msg/kinesis_video_frame.hpp>
#include <kinesis_webrtc_manager/common.h>
#include <kinesis_webrtc_manager/kinesis_webrtc_utils.h>

#include "kinesis_webrtc_streamer/subscriber_callbacks.h"

#include <iostream>
#include <string>

namespace Aws {
namespace Kinesis {

WebRtcNode::WebRtcNode(const std::string & node_name, const rclcpp::NodeOptions & options)
  : rclcpp::Node(node_name, options),
    webrtc_manager_(nullptr)
{
}

KinesisWebRtcManagerStatus WebRtcNode::Initialize(
  std::shared_ptr<Aws::Client::ParameterReaderInterface> param_reader,
  std::shared_ptr<KinesisWebRtcManagerInterface> webrtc_manager,
  std::vector<StreamerParams> vector_streamer_params)
{
  InitializeStreamerParameters(param_reader, vector_streamer_params);

  std::vector<WebRtcStreamInfo> webrtc_stream_infos;
  SetupWebRtcStreamInfos(vector_streamer_params, webrtc_stream_infos);

  webrtc_manager_ = webrtc_manager;
  ReadAwsConfig(param_reader, aws_config_);
  
  // Initialize the WebRTC manager with the stream infos
  KinesisWebRtcManagerStatus status = webrtc_manager_->InitializeWebRtc(webrtc_stream_infos);
  if (KinesisWebRtcManagerStatusSucceeded(status)) {
    InitializePublishers(vector_streamer_params);
    if (!SetupDefaultCallbacks(vector_streamer_params)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to setup default callbacks");
      return KinesisWebRtcManagerStatus::INVALID_INPUT;
    }
  }
  return status;
}

void WebRtcNode::PublishDataChannelMessage(const std::string & signaling_channel_name, const std::string & data) const
{
  if (publishers_.find(signaling_channel_name) != publishers_.end()) {
    auto message = std::make_unique<std_msgs::msg::String>();
    message->data = data;
    publishers_.at(signaling_channel_name)->publish(std::move(message));
  }
}

void WebRtcNode::InitializePublishers(const std::vector<StreamerParams> & vector_streamer_params)
{
  for (const auto & streamer_params : vector_streamer_params) {
    if (!streamer_params.publish_topic_.empty()) {
      publishers_[streamer_params.signaling_channel_name_] = 
        this->create_publisher<std_msgs::msg::String>(streamer_params.publish_topic_, 10);
    }
  }
}

void WebRtcNode::InitializeStreamerParameters(std::shared_ptr<Aws::Client::ParameterReaderInterface> param_reader,
                                             std::vector<StreamerParams> & vector_streamer_params)
{
  ReadStreamerParams(param_reader, vector_streamer_params);
}

void WebRtcNode::SetupWebRtcStreamInfos(const std::vector<StreamerParams> & vector_streamer_params,
                                       std::vector<WebRtcStreamInfo> & webrtc_stream_infos)
{
  for (const auto & streamer_params : vector_streamer_params) {
    SignalingChannelCustomData signaling_channel_custom_data;
    signaling_channel_custom_data.webrtc_node_ = this;
    signaling_channel_custom_data.signaling_channel_name_ = streamer_params.signaling_channel_name_;
    signaling_channel_custom_datas_.push_back(signaling_channel_custom_data);
    
    UINT64 custom_data = 0;
    RtcOnMessage callback = nullptr;
    
    if (!streamer_params.publish_topic_.empty()) {
      callback = OnDataChannelMessage;
      custom_data = reinterpret_cast<UINT64>(&signaling_channel_custom_datas_.back());
    }
    
    // Create a WebRtcStreamInfo object and add it to the vector
    webrtc_stream_infos.emplace_back(
      WebRtcStreamInfo{
        custom_data,
        streamer_params.signaling_channel_name_,
        streamer_params.video_format_type_,
        callback
      }
    );
  }
}

bool WebRtcNode::SetupDefaultCallbacks(const std::vector<StreamerParams> & vector_streamer_params)
{
  for (const auto & streamer_params : vector_streamer_params) {
    if (!streamer_params.video_subscription_topic_.empty()) {
      SetupKinesisVideoFrameTransport(streamer_params.signaling_channel_name_, 
                                     streamer_params.video_subscription_topic_);
    }
    
    if (!streamer_params.data_subscription_topic_.empty()) {
      SetupStringTransport(streamer_params.signaling_channel_name_, 
                          streamer_params.data_subscription_topic_);
    }
  }
  return true;
}

void WebRtcNode::SetupKinesisVideoFrameTransport(const std::string & signaling_channel_name,
                                               const std::string & subscription_topic)
{
  auto callback = [this, signaling_channel_name](const std::shared_ptr<kinesis_video_msgs::msg::KinesisVideoFrame> msg) {
    KinesisVideoFrameTransportCallback(*webrtc_manager_, signaling_channel_name, msg);
  };
  
  video_subscriptions_[signaling_channel_name] = 
    this->create_subscription<kinesis_video_msgs::msg::KinesisVideoFrame>(
      subscription_topic, 10, callback);
}

void WebRtcNode::SetupStringTransport(const std::string & signaling_channel_name,
                                    const std::string & subscription_topic)
{
  auto callback = [this, signaling_channel_name](const std::shared_ptr<std_msgs::msg::String> msg) {
    StringTransportCallback(*webrtc_manager_, signaling_channel_name, msg);
  };
  
  data_subscriptions_[signaling_channel_name] = 
    this->create_subscription<std_msgs::msg::String>(
      subscription_topic, 10, callback);
}

void OnDataChannelMessage(UINT64 custom_data, PRtcDataChannel p_rtc_data_channel, BOOL is_binary, PBYTE p_message, UINT32 p_message_len)
{
  (void)p_rtc_data_channel; // Unused parameter
  
  if (custom_data == 0 || p_message == nullptr || p_message_len == 0 || is_binary) {
    return;
  }
  
  auto signaling_channel_custom_data = reinterpret_cast<WebRtcNode::SignalingChannelCustomData*>(custom_data);
  if (signaling_channel_custom_data == nullptr || signaling_channel_custom_data->webrtc_node_ == nullptr) {
    return;
  }
  
  std::string message(reinterpret_cast<char*>(p_message), p_message_len);
  signaling_channel_custom_data->webrtc_node_->PublishDataChannelMessage(
    signaling_channel_custom_data->signaling_channel_name_, message);
}

} // namespace Kinesis
} // namespace Aws
