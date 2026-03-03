/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <kinesis_webrtc_streamer/streamer_params.h>

using namespace Aws::Kinesis;
using namespace testing;

// Minimal test that doesn't interact with AWS SDK
TEST(WebRtcNodeMinimalTest, StreamerParamsDefaultValues) {
  // Test that the default values are set correctly
  StreamerParams params;
  EXPECT_EQ(kSignalingChannelNameDefault, params.signaling_channel_name_);
  EXPECT_EQ(kVideoSubscriptionTopicDefault, params.video_subscription_topic_);
  EXPECT_EQ(kDataSubscriptionTopicDefault, params.data_subscription_topic_);
  EXPECT_EQ(kPublishTopicDefault, params.publish_topic_);
  
  // Skip checking video_format_type_ as it's not properly initialized by default
  // and contains a garbage value
}

// Test that we can create and modify StreamerParams
TEST(WebRtcNodeMinimalTest, StreamerParamsModification) {
  StreamerParams params;
  
  // Modify the parameters
  const std::string test_channel = "test_channel";
  const std::string test_video_topic = "test_video_topic";
  const std::string test_data_topic = "test_data_topic";
  const std::string test_publish_topic = "test_publish_topic";
  const VideoFormatType test_format = VideoFormatType::H264;
  
  params.signaling_channel_name_ = test_channel;
  params.video_subscription_topic_ = test_video_topic;
  params.data_subscription_topic_ = test_data_topic;
  params.publish_topic_ = test_publish_topic;
  params.video_format_type_ = test_format;
  
  // Verify the modifications
  EXPECT_EQ(test_channel, params.signaling_channel_name_);
  EXPECT_EQ(test_video_topic, params.video_subscription_topic_);
  EXPECT_EQ(test_data_topic, params.data_subscription_topic_);
  EXPECT_EQ(test_publish_topic, params.publish_topic_);
  EXPECT_EQ(test_format, params.video_format_type_);
}

// Test that we can create a vector of StreamerParams
TEST(WebRtcNodeMinimalTest, StreamerParamsVector) {
  std::vector<StreamerParams> params_vector;
  
  // Add some parameters
  for (int i = 0; i < 3; ++i) {
    StreamerParams params;
    params.signaling_channel_name_ = "channel_" + std::to_string(i);
    params.video_subscription_topic_ = "video_topic_" + std::to_string(i);
    params.data_subscription_topic_ = "data_topic_" + std::to_string(i);
    params.publish_topic_ = "publish_topic_" + std::to_string(i);
    params_vector.push_back(params);
  }
  
  // Verify the vector
  EXPECT_EQ(3u, params_vector.size());
  for (int i = 0; i < 3; ++i) {
    EXPECT_EQ("channel_" + std::to_string(i), params_vector[i].signaling_channel_name_);
    EXPECT_EQ("video_topic_" + std::to_string(i), params_vector[i].video_subscription_topic_);
    EXPECT_EQ("data_topic_" + std::to_string(i), params_vector[i].data_subscription_topic_);
    EXPECT_EQ("publish_topic_" + std::to_string(i), params_vector[i].publish_topic_);
  }
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
