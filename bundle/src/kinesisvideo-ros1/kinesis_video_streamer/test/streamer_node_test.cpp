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

#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <aws/core/Aws.h>
#include <kinesis_video_streamer/ros_stream_subscription_installer.h>
#include <kinesis_video_streamer/streamer.h>
#include <kinesis_video_streamer/subscriber_callbacks.h>
#include <ros/ros.h>

using ::testing::Eq;
using ::testing::Return;
using ::testing::AtLeast;
using Aws::AwsError;

using namespace Aws::Kinesis;

class MockRosStreamSubscriptionInstaller : public RosStreamSubscriptionInstaller
{
public:
  MockRosStreamSubscriptionInstaller(ros::NodeHandle & handle) : RosStreamSubscriptionInstaller(handle) {}
  
  MOCK_METHOD0(SetDefaultCallbacks, bool());
  MOCK_METHOD1(SetupImageTransport, bool(const ImageTransportCallbackFn callback));
  MOCK_METHOD1(set_stream_manager, void(KinesisStreamManagerInterface * stream_manager));
};

class TestStreamerNode : public ::testing::Test
{
protected:
  std::shared_ptr<MockRosStreamSubscriptionInstaller> mock_subscription_installer_;
  std::shared_ptr<StreamerNode> streamer_node_;
  
  void SetUp() override
  {
    streamer_node_ = std::make_shared<StreamerNode>("~");
    mock_subscription_installer_ = std::make_shared<MockRosStreamSubscriptionInstaller>(*streamer_node_);
    streamer_node_->set_subscription_installer(mock_subscription_installer_);
  }
  
  void TearDown() override
  {
    streamer_node_.reset();
    mock_subscription_installer_.reset();
  }
};

TEST_F(TestStreamerNode, CreateNode)
{
  EXPECT_TRUE(true);
}

TEST_F(TestStreamerNode, InitializeStreamerNode)
{
  EXPECT_CALL(*mock_subscription_installer_, SetDefaultCallbacks())
    .Times(AtLeast(1)).WillRepeatedly(Return(true));
  KinesisManagerStatus initialize_result = streamer_node_->Initialize();
  EXPECT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(initialize_result));
}

int main(int argc, char ** argv)
{
  Aws::SDKOptions options_;
  Aws::InitAPI(options_);
    
  testing::InitGoogleMock(&argc, argv);

  ros::init(argc, argv, "test_streamer_node");
  ros::NodeHandle n("test_streamer_node");
  n.setParam("aws_client_configuration/region", "us-west-2");
  int ret = RUN_ALL_TESTS();

  return ret;
}
