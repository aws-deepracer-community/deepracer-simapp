/*
 * Copyright Amazon.com, Inc. or its affiliates. All Rights Reserved.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <gmock/gmock.h>
#include <gtest/gtest.h>
#include <kinesis_webrtc_manager/kinesis_webrtc_manager.h>
#include <kinesis_webrtc_streamer/subscriber_callbacks.h>
#include <rclcpp/rclcpp.hpp>

using namespace Aws::Kinesis;
using namespace testing;
using ::testing::Return;
using ::testing::SaveArg;

// Create a completely mocked version of KinesisWebRtcManagerInterface
// that doesn't rely on any actual AWS SDK code
class StrictMockKinesisWebRtcManager : public KinesisWebRtcManagerInterface
{
public:
  StrictMockKinesisWebRtcManager() = default;
  virtual ~StrictMockKinesisWebRtcManager() = default;
  
  MOCK_METHOD1(
    InitializeWebRtc,
    KinesisWebRtcManagerStatus(const std::vector<WebRtcStreamInfo> & stream_infos)
  );
  MOCK_CONST_METHOD2(
    PutFrame,
    KinesisWebRtcManagerStatus(const std::string & signaling_channel_name, const Frame * frame)
  );
  MOCK_CONST_METHOD4(
    SendDataMessage,
    KinesisWebRtcManagerStatus(const std::string & signaling_channel_name,
                               const bool is_binary,
                               const PBYTE data,
                               const UINT32 length)
  );
};

class SubscriberCallbacksTestFixture : public ::testing::Test
{
protected:
  std::shared_ptr<StrictMockKinesisWebRtcManager> mock_webrtc_manager_;
  std::shared_ptr<kinesis_video_msgs::msg::KinesisVideoFrame> video_msg_;
  std::shared_ptr<std_msgs::msg::String> string_msg_;
  std::string test_signaling_channel_name = "test_signaling_channel_name";

  void SetUp() override
  {
    mock_webrtc_manager_ = std::make_shared<StrictMockKinesisWebRtcManager>();
    video_msg_ = std::make_shared<kinesis_video_msgs::msg::KinesisVideoFrame>();
    string_msg_ = std::make_shared<std_msgs::msg::String>();
  }
  
  void TearDown() override
  {
    string_msg_.reset();
    video_msg_.reset();
    mock_webrtc_manager_.reset();
  }
};

TEST_F(SubscriberCallbacksTestFixture, KinesisVideoFrameTransportCallbackSuccess)
{
  EXPECT_CALL(*mock_webrtc_manager_, PutFrame(_,_))
    .Times(1)
    .WillOnce(Return(KinesisWebRtcManagerStatus::SUCCESS));

  KinesisVideoFrameTransportCallback(*mock_webrtc_manager_, test_signaling_channel_name, video_msg_);
}

TEST_F(SubscriberCallbacksTestFixture, KinesisVideoFrameTransportCallbackMissingSignalingChannel)
{
  EXPECT_CALL(*mock_webrtc_manager_, PutFrame(_,_))
    .Times(1)
    .WillOnce(Return(KinesisWebRtcManagerStatus::PUTFRAME_SIGNALING_CHANNEL_NOT_FOUND));
  
  KinesisVideoFrameTransportCallback(*mock_webrtc_manager_, test_signaling_channel_name, video_msg_);
}

TEST_F(SubscriberCallbacksTestFixture, KinesisVideoFrameTransportCallbackKeyFrame)
{
  EXPECT_CALL(*mock_webrtc_manager_, PutFrame(_,_))
    .Times(1)
    .WillOnce(Return(KinesisWebRtcManagerStatus::SUCCESS));

  video_msg_->flags = 1;
  KinesisVideoFrameTransportCallback(*mock_webrtc_manager_, test_signaling_channel_name, video_msg_);
}

TEST_F(SubscriberCallbacksTestFixture, KinesisVideoFrameTransportCallbackNoViewer)
{
  EXPECT_CALL(*mock_webrtc_manager_, PutFrame(_,_))
    .Times(1)
    .WillOnce(Return(KinesisWebRtcManagerStatus::PUTFRAME_NO_VIEWER));
  
  KinesisVideoFrameTransportCallback(*mock_webrtc_manager_, test_signaling_channel_name, video_msg_);
}

TEST_F(SubscriberCallbacksTestFixture, StringTransportCallbackSuccess)
{
  EXPECT_CALL(*mock_webrtc_manager_, SendDataMessage(_,_,_,_))
    .Times(1)
    .WillOnce(Return(KinesisWebRtcManagerStatus::SUCCESS));

  StringTransportCallback(*mock_webrtc_manager_, test_signaling_channel_name, string_msg_);  
}

TEST_F(SubscriberCallbacksTestFixture, StringTransportCallbackFailure)
{
  EXPECT_CALL(*mock_webrtc_manager_, SendDataMessage(_,_,_,_))
    .Times(1)
    .WillOnce(Return(KinesisWebRtcManagerStatus::INVALID_INPUT));

  StringTransportCallback(*mock_webrtc_manager_, test_signaling_channel_name, string_msg_);  
}

TEST_F(SubscriberCallbacksTestFixture, StringTransportCallbackNoViewer)
{
  EXPECT_CALL(*mock_webrtc_manager_, SendDataMessage(_,_,_,_))
    .Times(1)
    .WillOnce(Return(KinesisWebRtcManagerStatus::SENDDATAMESSAGE_NO_VIEWER));

  StringTransportCallback(*mock_webrtc_manager_, test_signaling_channel_name, string_msg_);  
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  rclcpp::init(argc, argv);
  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
