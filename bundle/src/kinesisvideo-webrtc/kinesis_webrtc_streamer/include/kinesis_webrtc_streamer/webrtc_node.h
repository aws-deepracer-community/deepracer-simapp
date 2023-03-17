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
#include <set>
#include <string>
#include <vector>

#include <aws_ros1_common/sdk_utils/ros1_node_parameter_reader.h>
#include <kinesis_video_msgs/KinesisVideoFrame.h>
#include <kinesis_webrtc_manager/kinesis_webrtc_manager.h>
#include <ros/ros.h>

#include "kinesis_webrtc_streamer/streamer_params.h"
#include "kinesis_webrtc_streamer/subscriber_callbacks.h"

namespace Aws {
namespace Kinesis {

/**
 * ROS-less free function callback that gets called when this WebRtcNode receives a data message 
 * from any of its connected viewers and calls PublishDataChannelMessage with the necessary information
 * to publish the message within the ROS ecosystem.
 * @param custom_data
 * @param p_rtc_data_channel
 * @param is_binary
 * @param p_message
 * @param p_message_len
 */
void OnDataChannelMessage(UINT64 custom_data, PRtcDataChannel p_rtc_data_channel, BOOL is_binary, PBYTE p_message, UINT32 p_message_len);

/**
 * ROS1 specific node that gets run to setup the WebRTC connection, ROS publishers, and ROS subscriptions.
 */
class WebRtcNode : public ros::NodeHandle
{
public:
  WebRtcNode(const std::string & ns);
  
  ~WebRtcNode() = default;

  /**
   * Sets up the WebRTC connection through the use of the kinesis_webrtc_manager.
   * Initializes the publisher(s) and subscription(s) which are specified in deepracer_webrtc_config.yaml.
   * @param param_reader
   * @param webrtc_manager
   * @param vector_streamer_params
   */
  KinesisWebRtcManagerStatus Initialize(
    std::shared_ptr<Aws::Client::ParameterReaderInterface> param_reader,
    std::shared_ptr<KinesisWebRtcManagerInterface> webrtc_manager = std::make_shared<KinesisWebRtcManager>(),
    std::vector<StreamerParams> vector_streamer_params = std::vector<StreamerParams>());
  
  /**
   * Gets called by OnDataChannelMessage() to publish the string message received over the WebRTC connection.
   * Note, only publishes the data message if the signaling channel exists.
   * @param signaling_channel_name
   * @param data
   */
  void PublishDataChannelMessage(const std::string & signaling_channel_name, const std::string & data) const;
  
  /**
   * Structure that will be passed into the WebRTC SDK in C as custom_data, which
   * then gets accessed within the OnDataChannelMessage() free function.
   */
  struct SignalingChannelCustomData {
    WebRtcNode *webrtc_node_;
    std::string signaling_channel_name_;
  };
private:
  std::unordered_map<std::string, ros::Subscriber> video_subscriptions_;
  std::unordered_map<std::string, ros::Subscriber> data_subscriptions_;
  std::unordered_map<std::string, ros::Publisher> publishers_;
  std::shared_ptr<KinesisWebRtcManagerInterface> webrtc_manager_;

  Aws::Client::ClientConfiguration aws_config_;

  /**
   * Initializes the std_msgs::msg::String publisher(s) for when the WebRtcNode receives a data message 
   * over the WebRTC Peer-to-Peer connection for each signaling_channel_name and publish_topic_name passed in.
   * Note, multiple signaling channels can have the same corresponding publish topic.
   * @param vector_streamer_params
   */
  void InitializePublishers(const std::vector<StreamerParams> & vector_streamer_params);

  /**
   * Initializes the parameters for the WebRtcNode by using the functions declared within streamer_params.h.
   * @param param_reader
   * @param[out] vector_streamer_params
   */
  void InitializeStreamerParameters(std::shared_ptr<Aws::Client::ParameterReaderInterface> param_reader,
                                    std::vector<StreamerParams> & vector_streamer_params);

  /**
   * Sets up the vector of WebRtcStreamInfo structs which is used by the kinesis_webrtc_manager
   * to initialize the WebRTC connection(s).
   * @param vector_streamer_params
   * @param[out] webrtc_stream_infos
   */
  void SetupWebRtcStreamInfos(const std::vector<StreamerParams> & vector_streamer_params,
                              std::vector<WebRtcStreamInfo> & webrtc_stream_infos);

  /**
   * Initializes the subscription(s) for each signaling channel as long as the parameter is not empty.
   * Calls SetupKinesisVideoFrameTransport() and SetupStringTransport().
   * Note, multiple signaling channels can subscribe to the same ROS topic.
   * @param vector_streamer_params
   */
  bool SetupDefaultCallbacks(const std::vector<StreamerParams> & vector_streamer_params);

  /**
   * Creates ROS subscription to the subscription topic name passed in using the
   * ROS-specific callback declared in subscriber_callbacks.h 
   * which handles a kinesis_video_msgs::msg::KinesisVideoFrame input.
   * Maintains which signaling channel coresponds to which subscription with the use of the 
   * std::unordered_map video_subscriptions_.
   * @param signaling_channel_name
   * @param subscription_topic
   */
  void SetupKinesisVideoFrameTransport(const std::string & signaling_channel_name,
                                       const std::string & subscription_topic);
	   
   /**
    * Creates ROS subscription to the subscription topic name passed in.
    * Maintains which signaling channel coresponds to which subscription with the use of the 
    * std::unordered_map video_subscriptions_.
    * This function will be used when the data_subscription_type parameter is "std_msgs/msg/String"
    * @param signaling_channel_name
    * @param subscription_topic
    */
   void SetupStringTransport(const std::string & signaling_channel_name,
                             const std::string & subscription_topic);

  std::vector<SignalingChannelCustomData> signaling_channel_custom_datas_; 
};

} // namespace Kinesis
} // namespace Aws
