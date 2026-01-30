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
#include <aws/core/Aws.h>
#include <aws/core/utils/logging/LogMacros.h>
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <kinesis_manager/kinesis_stream_manager.h>
#include <kinesis_manager/stream_definition_provider.h>
#include "kinesis_video_msgs/msg/kinesis_video_frame.hpp"
#include <kinesis_video_streamer/ros_stream_subscription_installer.h>
#include <kinesis_video_streamer/streamer.h>
#include <kinesis_video_streamer/subscriber_callbacks.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <rclcpp/qos.hpp>

#include <queue>
#include <gmock/gmock.h>

#include "kinesis_video_streamer_test_utils.h"

using namespace std;
using namespace Aws::Client;
using namespace Aws::Kinesis;
using namespace Aws::Utils::Logging;
using namespace testing;

TestData * kTestData = nullptr;
AWSROSLogger * kLogger;
const char * kTopicName = "/rekognition_results";
const uint16_t kQueueDepth = 10;

class MockKinesisStreamManager : public KinesisStreamManager
{
public:
  MockKinesisStreamManager(Aws::Client::ParameterReaderInterface * parameter_reader,
                           StreamDefinitionProvider * stream_definition_provider,
                                StreamSubscriptionInstaller * subscription_installer, std::unique_ptr<KinesisClient> kinesis_client) : KinesisStreamManager(parameter_reader,
 stream_definition_provider, subscription_installer, std::move(kinesis_client)) {}

  MOCK_METHOD2(ProcessCodecPrivateDataForStream, KinesisManagerStatus(const std::string & stream_name, std::vector<uint8_t> codec_private_data));
  MOCK_CONST_METHOD2(PutFrame, KinesisManagerStatus(std::string stream_name, Frame & frame));
  MOCK_CONST_METHOD3(PutMetadata, KinesisManagerStatus(std::string stream_name, const std::string & name, const std::string & value));
  MOCK_METHOD2(FetchRekognitionResults, KinesisManagerStatus(const std::string & stream_name, Aws::Vector<Model::Record> * records));
};

class SubscriberCallbacksTestBase : public ::testing::Test
{
protected:
    void SetUp() override
    {
        handle = rclcpp::Node::make_shared("test_node");
        pub_ = handle->create_publisher<std_msgs::msg::String>(kTopicName, rclcpp::QoS(10));
        auto callback = [this](const std_msgs::msg::String::SharedPtr msg) -> void {
            this->rekognition_results.push_back(msg->data);
        };
        sub_ = handle->create_subscription<std_msgs::msg::String>(kTopicName, kQueueDepth, callback);
        kTestData = &test_data;

        stream_definition_provider = new MockStreamDefinitionProvider(&test_data);
        subscription_installer_ = std::make_shared<MockStreamSubscriptionInstaller>(&test_data, *stream_manager, handle);

        Aws::Client::ClientConfiguration clientConfig;
        clientConfig.region = Aws::Region::US_WEST_2;
        auto kinesis_client = std::make_unique<Aws::Kinesis::KinesisClient>(clientConfig);
        stream_manager = std::make_shared<MockKinesisStreamManager>(&parameter_reader,
                                                      stream_definition_provider, subscription_installer_.get(), std::move(kinesis_client));
        message_ = std::make_shared<kinesis_video_msgs::msg::KinesisVideoFrame>();
    }

    void TearDown() override
    {
        stream_manager.reset();
        delete stream_definition_provider;
        subscription_installer_.reset();
        message_.reset();
    }

    std::vector<std::string> rekognition_results;
    std::shared_ptr<rclcpp::Node> handle;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
    TestParameterReader parameter_reader;
    TestData test_data;
    std::shared_ptr<MockKinesisStreamManager> stream_manager;
    StreamDefinitionProvider real_stream_definition_provider;
    MockStreamDefinitionProvider * stream_definition_provider;
    std::shared_ptr<MockStreamSubscriptionInstaller> subscription_installer_;
    std::shared_ptr<kinesis_video_msgs::msg::KinesisVideoFrame> message_;
};

/**
 * Tests success scenario.
 */
TEST_F(SubscriberCallbacksTestBase, KinesisVideoFrameTransportCallbackSuccessTest)
{
    message_->index = 1;
    message_->flags = 2;

    /* No codec data and no metadata */
    EXPECT_CALL(*stream_manager, ProcessCodecPrivateDataForStream(_, _)).Times(0);
    EXPECT_CALL(*stream_manager, PutFrame(_, _)).Times(1);
    EXPECT_CALL(*stream_manager, PutMetadata(_, _, _)).Times(0);
    KinesisVideoFrameTransportCallback(*stream_manager, "stream_name", message_);

    /* Codec data exists */
    message_->codec_private_data.push_back(0xde);
    EXPECT_CALL(*stream_manager, ProcessCodecPrivateDataForStream(_, _)).Times(1);
    EXPECT_CALL(*stream_manager, PutFrame(_, _)).Times(1);
    EXPECT_CALL(*stream_manager, PutMetadata(_, _, _)).Times(0);
    KinesisVideoFrameTransportCallback(*stream_manager, "stream_name", message_);

    diagnostic_msgs::msg::KeyValue val;
    message_->metadata.push_back(val);
    /* Metadata exists */
    EXPECT_CALL(*stream_manager, ProcessCodecPrivateDataForStream(_, _)).Times(1);
    EXPECT_CALL(*stream_manager, PutFrame(_, _)).Times(1);
    EXPECT_CALL(*stream_manager, PutMetadata(_, _, _)).Times(1);
    KinesisVideoFrameTransportCallback(*stream_manager, "stream_name", message_);
}

/**
 * Tests failure scenarios.
 *  Failures inside callbacks are usually not fatal; we continue as best effort.
 */
TEST_F(SubscriberCallbacksTestBase, KinesisVideoFrameTransportCallbackFailuresTest)
{

    /* 1. Fail ProcessCodecPrivateDataForStream and make sure PutFrame still gets called. */
    /* 2. Fail PutFrame and make sure PutMetadata still gets called */
    /* 3. PutMetadata with two KeyValues: failure on the 1st one should not prevent calling PutMetadata for the 2nd. */
    ON_CALL(*stream_manager, ProcessCodecPrivateDataForStream(_, _))
        .WillByDefault(Return(KINESIS_MANAGER_STATUS_PROCESS_CODEC_DATA_STREAM_CONFIG_NOT_FOUND));
    ON_CALL(*stream_manager, PutFrame(_, _))
        .WillByDefault(Return(KINESIS_MANAGER_STATUS_PUTFRAME_FAILED));
    ON_CALL(*stream_manager, PutMetadata(_, _, _))
        .WillByDefault(Return(KINESIS_MANAGER_STATUS_PUTMETADATA_FAILED));

    EXPECT_CALL(*stream_manager, ProcessCodecPrivateDataForStream(_, _))
        .Times(3)
        .WillRepeatedly(Return(KINESIS_MANAGER_STATUS_PROCESS_CODEC_DATA_STREAM_CONFIG_NOT_FOUND));;
    EXPECT_CALL(*stream_manager, PutFrame(_, _))
        .Times(3)
        .WillRepeatedly(Return(KINESIS_MANAGER_STATUS_PUTFRAME_FAILED));
    EXPECT_CALL(*stream_manager, PutMetadata(_, _, _))
        .Times(3)
        .WillRepeatedly(Return(KINESIS_MANAGER_STATUS_PUTMETADATA_FAILED));

    message_->codec_private_data.push_back(0xde);
    KinesisVideoFrameTransportCallback(*stream_manager, "stream_name", message_);

    diagnostic_msgs::msg::KeyValue val;
    message_->metadata.push_back(val);
    KinesisVideoFrameTransportCallback(*stream_manager, "stream_name", message_);

    message_->metadata.push_back(val);
    KinesisVideoFrameTransportCallback(*stream_manager, "stream_name", message_);
}

/*
 * Tests Rekognition callback.
 */
TEST_F(SubscriberCallbacksTestBase, RekognitionCallbackTest)
{

    /* Should behave similarly to KinesisVideoFrameTransportCallback
         (since it directly invokes it after handling the Rekogntion-specific part). */
    ON_CALL(*stream_manager, ProcessCodecPrivateDataForStream(_, _))
        .WillByDefault(Return(KINESIS_MANAGER_STATUS_PROCESS_CODEC_DATA_STREAM_CONFIG_NOT_FOUND));
    ON_CALL(*stream_manager, PutFrame(_, _))
        .WillByDefault(Return(KINESIS_MANAGER_STATUS_PUTFRAME_FAILED));
    ON_CALL(*stream_manager, PutMetadata(_, _, _))
        .WillByDefault(Return(KINESIS_MANAGER_STATUS_PUTMETADATA_FAILED));

    EXPECT_CALL(*stream_manager, ProcessCodecPrivateDataForStream(_, _))
        .Times(1)
        .WillRepeatedly(Return(KINESIS_MANAGER_STATUS_PROCESS_CODEC_DATA_STREAM_CONFIG_NOT_FOUND));;
    EXPECT_CALL(*stream_manager, PutFrame(_, _))
        .Times(1)
        .WillRepeatedly(Return(KINESIS_MANAGER_STATUS_PUTFRAME_FAILED));
    EXPECT_CALL(*stream_manager, PutMetadata(_, _, _))
        .Times(2)
        .WillRepeatedly(Return(KINESIS_MANAGER_STATUS_PUTMETADATA_FAILED));

    ON_CALL(*stream_manager, FetchRekognitionResults(_, _)).WillByDefault(Return(KINESIS_MANAGER_STATUS_SUCCESS));
    EXPECT_CALL(*stream_manager, FetchRekognitionResults(_, _))
        .Times(1)
        .WillRepeatedly(Return(KINESIS_MANAGER_STATUS_SUCCESS));

    message_->codec_private_data.push_back(0xde);
    diagnostic_msgs::msg::KeyValue val;
    message_->metadata.push_back(val);
    message_->metadata.push_back(val);
    RekognitionEnabledKinesisVideoFrameTransportCallback(*stream_manager, "stream_name", message_, pub_);
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);

    Aws::SDKOptions options;
    Aws::InitAPI(options);

    int ret = RUN_ALL_TESTS();

    // Add this line to properly clean up ROS
    rclcpp::shutdown();

    Aws::Utils::Logging::ShutdownAWSLogging();
    Aws::ShutdownAPI(options);
    return ret;
}
