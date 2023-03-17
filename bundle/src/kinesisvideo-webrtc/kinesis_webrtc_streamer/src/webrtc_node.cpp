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
#include "kinesis_webrtc_streamer/webrtc_node.h"

#include <memory>

#include "std_msgs/String.h"
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_common/sdk_utils/client_configuration_provider.h>
#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <kinesis_video_msgs/KinesisVideoFrame.h>
#include <kinesis_webrtc_manager/common.h>
#include <kinesis_webrtc_manager/kinesis_webrtc_utils.h>

#include "kinesis_webrtc_streamer/subscriber_callbacks.h"

#include <iostream>
#include <string>

namespace Aws {
namespace Kinesis {

WebRtcNode::WebRtcNode(const std::string & ns)
  : NodeHandle(ns),
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
  
  AWS_LOGSTREAM_INFO(__func__, "vector_streamer_params size: " << std::to_string(vector_streamer_params.size()));
  KinesisWebRtcManagerStatus initialize_webrtc_result = webrtc_manager_->InitializeWebRtc(webrtc_stream_infos);
  if (KinesisWebRtcManagerStatusFailed(initialize_webrtc_result)) {
    AWS_LOGSTREAM_FATAL(__func__, "Failed to initialize WebRTC.");
    return initialize_webrtc_result;
  }

  InitializePublishers(vector_streamer_params);

  if (!SetupDefaultCallbacks(vector_streamer_params)) {
    AWS_LOG_FATAL(__func__, "Failed the set up default subscription callbacks.");
    return KinesisWebRtcManagerStatus::ERROR_BASE;
  }
  return KinesisWebRtcManagerStatus::SUCCESS;
}

void WebRtcNode::InitializeStreamerParameters(std::shared_ptr<Aws::Client::ParameterReaderInterface> param_reader,
                                              std::vector<StreamerParams> & vector_streamer_params)
{
  ReadAwsConfig(param_reader, aws_config_);
  ReadStreamerParams(param_reader, vector_streamer_params);
}

void WebRtcNode::InitializePublishers(const std::vector<StreamerParams> & vector_streamer_params)
{
  for (StreamerParams streamer_params : vector_streamer_params) {
    if (streamer_params.publish_topic_ != "") {
      publishers_.insert({streamer_params.signaling_channel_name_,
                          advertise<std_msgs::String>(streamer_params.publish_topic_, 10)
                         });
      AWS_LOGSTREAM_INFO(__func__, "Successfully created publisher for signaling_channel_name: "
        << streamer_params.signaling_channel_name_ << ". publish_topic: " << streamer_params.publish_topic_);
    } else {
      AWS_LOGSTREAM_INFO(__func__, "Did NOT create publisher for signaling_channel_name: " 
        << streamer_params.signaling_channel_name_ << ". publish_topic: " << streamer_params.publish_topic_);
    }
  }
}

void WebRtcNode::SetupWebRtcStreamInfos(const std::vector<StreamerParams> & vector_streamer_params,
                                        std::vector<WebRtcStreamInfo> & webrtc_stream_infos)
{
  signaling_channel_custom_datas_.reserve(vector_streamer_params.size());
  for (StreamerParams streamer_params : vector_streamer_params) {
    // Create a struct of `this` WebRtcNode and stream_index and pass it in as the custom_data
    // part of the C-style API, which will get passed into OnDataChannelMessage()'s custom_data parameter
    SignalingChannelCustomData signaling_channel_custom_data = {this, streamer_params.signaling_channel_name_};

    signaling_channel_custom_datas_.push_back(signaling_channel_custom_data);
    
    WebRtcStreamInfo webrtc_stream_info = {
      reinterpret_cast<UINT64>(&signaling_channel_custom_datas_.back()),
      streamer_params.signaling_channel_name_,
      streamer_params.video_format_type_,
      OnDataChannelMessage
    };
    webrtc_stream_infos.push_back(webrtc_stream_info);
  }
}

bool WebRtcNode::SetupDefaultCallbacks(const std::vector<StreamerParams> & vector_streamer_params)
{
  if (vector_streamer_params.size() == 0) {
    AWS_LOG_ERROR(__func__, "Failed to setup default callbacks. streamer_params is empty.");
    return false;
  }
  for (StreamerParams streamer_params : vector_streamer_params) {
    if (streamer_params.video_subscription_topic_ != "") {
      SetupKinesisVideoFrameTransport(
        streamer_params.signaling_channel_name_,
        streamer_params.video_subscription_topic_
      );
    }
    if (streamer_params.data_subscription_topic_ != "") {
      SetupStringTransport(
        streamer_params.signaling_channel_name_,
        streamer_params.data_subscription_topic_
      );
    }
  }
  return true;
}

void WebRtcNode::SetupKinesisVideoFrameTransport(
  const std::string & signaling_channel_name,
  const std::string & subscription_topic)
{
  boost::function<void(const kinesis_video_msgs::KinesisVideoFrame::ConstPtr &)> on_video_frame_received;
  
  on_video_frame_received = [this, signaling_channel_name] 
    (const kinesis_video_msgs::KinesisVideoFrame::ConstPtr & msg) -> void
    {
        KinesisVideoFrameTransportCallback(*this->webrtc_manager_, signaling_channel_name, msg);
    };
    video_subscriptions_.insert({signaling_channel_name,
                                 subscribe(subscription_topic, 10, on_video_frame_received)
                                });
}

void WebRtcNode::SetupStringTransport(
  const std::string & signaling_channel_name,
  const std::string & subscription_topic)
{
  boost::function<void(const std_msgs::String::ConstPtr &)> on_string_message_received;

  on_string_message_received = [this, signaling_channel_name](const std_msgs::String::ConstPtr & msg) -> void
    {
        StringTransportCallback(*this->webrtc_manager_, signaling_channel_name, msg->data);
    };
  data_subscriptions_.insert({signaling_channel_name,
                              subscribe(subscription_topic, 10, on_string_message_received)
                             });
}

void OnDataChannelMessage(UINT64 custom_data, PRtcDataChannel p_rtc_data_channel, BOOL is_binary, PBYTE p_message, UINT32 p_message_len)
{
  (void) is_binary;
  std::string data(reinterpret_cast<const char *>(p_message), p_message_len);

  AWS_LOGSTREAM_INFO(__func__, "Received DataChannel String Message: " << data);
  
  auto signaling_channel_custom_data = reinterpret_cast<WebRtcNode::SignalingChannelCustomData *>(custom_data);

  WebRtcNode * self = signaling_channel_custom_data->webrtc_node_;

  self->PublishDataChannelMessage(
    signaling_channel_custom_data->signaling_channel_name_,
    data
  );
}

void WebRtcNode::PublishDataChannelMessage(
  const std::string & signaling_channel_name,
  const std::string & data) const
{
  std_msgs::String string_msg;
  string_msg.data = data;

  auto entry = publishers_.find(signaling_channel_name);

  if (entry != publishers_.end()) {
    publishers_.at(signaling_channel_name).publish(string_msg);
  } else {
    AWS_LOGSTREAM_WARN(__func__, "No corresponding publisher for signaling channel: " << signaling_channel_name);
  }
}

} // namespace Kinesis
} // namespace Aws
