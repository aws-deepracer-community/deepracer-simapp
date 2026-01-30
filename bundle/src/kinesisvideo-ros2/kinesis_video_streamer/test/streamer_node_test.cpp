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

#include <gmock/gmock.h>

#include <aws/core/Aws.h>
#include <kinesis_video_streamer/ros_stream_subscription_installer.h>
#include <kinesis_video_streamer/streamer.h>
#include <kinesis_video_streamer/subscriber_callbacks.h>
#include <rclcpp/rclcpp.hpp>

using ::testing::Eq;
using ::testing::Return;
using ::testing::AtLeast;
using Aws::AwsError;

using namespace Aws::Kinesis;

class MockRosStreamSubscriptionInstaller : public RosStreamSubscriptionInstaller
{
public:
  MockRosStreamSubscriptionInstaller(std::shared_ptr<rclcpp::Node> handle) : RosStreamSubscriptionInstaller(handle) {}
  
  MOCK_METHOD0(SetDefaultCallbacks, bool());
  MOCK_METHOD1(SetupImageTransport, bool(const ImageTransportCallbackFn callback));
  MOCK_METHOD1(set_stream_manager, void(KinesisStreamManagerInterface * stream_manager));
};

class TestStreamerNode : public ::testing::Test
{
protected:
  std::shared_ptr<MockRosStreamSubscriptionInstaller> mock_subscription_installer_;
  std::shared_ptr<StreamerNode> streamer_node_;
  std::shared_ptr<Aws::Client::Ros2NodeParameterReader> parameter_reader_;
  rclcpp::NodeOptions node_options_;

  void SetUp() override
  {
    node_options_.allow_undeclared_parameters(true);
    node_options_.automatically_declare_parameters_from_overrides(true);
    streamer_node_ = std::make_shared<StreamerNode>("streamer", std::string(), node_options_);
    parameter_reader_ = std::make_shared<Aws::Client::Ros2NodeParameterReader>(streamer_node_);
    mock_subscription_installer_ = std::make_shared<MockRosStreamSubscriptionInstaller>(streamer_node_);
  }
  
  void TearDown() override
  {
    streamer_node_.reset();
    mock_subscription_installer_.reset();
  }
};

TEST_F(TestStreamerNode, InitializeStreamerNode)
{
  EXPECT_CALL(*mock_subscription_installer_, SetDefaultCallbacks())
    .Times(0);
  KinesisManagerStatus initialize_result = streamer_node_->Initialize(parameter_reader_, mock_subscription_installer_);
  /* Region is not set */
  EXPECT_FALSE(KINESIS_MANAGER_STATUS_SUCCEEDED(initialize_result));
  EXPECT_EQ(initialize_result, KINESIS_MANAGER_STATUS_INVALID_INPUT);
  /* Set the region and expect success */
  rclcpp::Parameter region("aws_client_configuration.region", "not-empty");
  streamer_node_->set_parameters({region});
  initialize_result = streamer_node_->Initialize(parameter_reader_, mock_subscription_installer_);
  EXPECT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(initialize_result));
  /* InitializeStreamSubscriptions should fail as we have not set the stream count and type parameters */
  initialize_result = streamer_node_->InitializeStreamSubscriptions();
  EXPECT_FALSE(KINESIS_MANAGER_STATUS_SUCCEEDED(initialize_result));
}

int main(int argc, char ** argv)
{
  Aws::SDKOptions options_;
  Aws::InitAPI(options_);
  testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);
  int ret = RUN_ALL_TESTS();

  // Add these lines to properly clean up
  rclcpp::shutdown();
  Aws::ShutdownAPI(options_);

  return ret;
}
