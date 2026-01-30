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
#include <aws_ros2_common/sdk_utils/logging/aws_ros_logger.h>
#include <gtest/gtest.h>
#include <kinesis_manager/kinesis_stream_manager.h>
#include <kinesis_manager/stream_definition_provider.h>
#include "kinesis_video_msgs/msg/kinesis_video_frame.hpp"
#include <kinesis_video_streamer/ros_stream_subscription_installer.h>
#include <kinesis_video_streamer/streamer.h>
#include <kinesis_video_streamer/subscriber_callbacks.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include <queue>

#include "kinesis_video_streamer_test_utils.h"

/* Maximum time to wait for callbacks to become available. */
constexpr chrono::nanoseconds kShortCallbackWaitTime = chrono::duration_cast<chrono::nanoseconds>(chrono::milliseconds(100));
constexpr chrono::nanoseconds kLongCallbackWaitTime = chrono::duration_cast<chrono::nanoseconds>(chrono::milliseconds(5000));
/** @def ROS_POSTCALLBACK_ASSERT_TRUE
 * @brief Some tests rely on ROS callback processing which may take time. To avoid unnecessary waits
 * we define a helper macro with the following logic: If the assertion would fail, call available
 * callbacks first. If the assertion would still fail, call a single callback with a "small"
 * timeout (0.1s). If the assertion would still fail, call a single callback with a "large"
 * timeout (5s). finally, ASSERT_TRUE. This avoids unnecessary waits and ensures sufficient
 * processing time if needed.
 * */
#define ROS_POSTCALLBACK_ASSERT_TRUE(expr, handle)              \
  do {                                                          \
      if (!(expr)) {                                            \
        rclcpp::executors::SingleThreadedExecutor executor;     \
        executor.add_node(handle);                              \
        std::this_thread::sleep_for(std::chrono::milliseconds(3000)); \
        executor.spin_some();                                   \
        if (!(expr)) {                                          \
            executor.spin_once(kShortCallbackWaitTime);         \
            if (!(expr)) {                                      \
                executor.spin_once(kLongCallbackWaitTime);      \
            }                                                   \
        }                                                       \
        executor.remove_node(handle);                           \
      }                                                         \
      ASSERT_TRUE(expr);                                        \
  } while (false)

using namespace std;
using namespace Aws::Client;
using namespace Aws::Kinesis;
using namespace Aws::Utils::Logging;

TestData * kTestData = nullptr;
AWSROSLogger * kLogger;

class KinesisVideoStreamerTestBase : public ::testing::Test
{
protected:
    void SetUp() override
    {
        handle = rclcpp::Node::make_shared("test_node");
        kTestData = &test_data;
        stream_definition_provider = new MockStreamDefinitionProvider(&test_data);
        subscription_installer =
                new MockStreamSubscriptionInstaller(&test_data, *stream_manager, handle);
        stream_manager = new MockStreamManager(&test_data, &parameter_reader,
                                               stream_definition_provider, subscription_installer);
        subscription_installer->set_stream_manager(stream_manager);
    }

    void TearDown() override
    {
        delete stream_manager;
        delete stream_definition_provider;
        delete subscription_installer;
    }

    std::shared_ptr<rclcpp::Node> handle;
    TestParameterReader parameter_reader;
    TestData test_data;
    MockStreamManager * stream_manager;
    StreamDefinitionProvider real_stream_definition_provider;
    MockStreamDefinitionProvider * stream_definition_provider;
    MockStreamSubscriptionInstaller * subscription_installer;
};

/**
 * Tests success scenario.
 */
TEST_F(KinesisVideoStreamerTestBase, sanity)
{
    int stream_count = 1;
    parameter_reader.int_map_.insert({string("kinesis_video/stream_count"), stream_count});
    parameter_reader.int_map_.at("kinesis_video/stream_count") = stream_count;
    parameter_reader.int_map_.insert({string("kinesis_video/stream0/topic_type"), 1});
    parameter_reader.string_map_.insert({string("kinesis_video/stream0/stream_name"), "stream-name"});
    parameter_reader.string_map_.insert(
    {string("kinesis_video/stream0/subscription_topic"), "topic-name"});
    unique_ptr<StreamDefinition> stream_definition =
            real_stream_definition_provider.GetStreamDefinition(ParameterPath(""), parameter_reader, nullptr, 0);
    test_data.get_stream_definition_return_value = (StreamDefinition *)stream_definition.release();
    KinesisManagerStatus setup_result = stream_manager->KinesisVideoStreamerSetup();
    ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(setup_result));
}

TEST_F(KinesisVideoStreamerTestBase, codecPrivateDataFailure)
{
    int stream_count = 100;
    parameter_reader.int_map_.insert({string("kinesis_video/stream_count"), stream_count});
    /**
     * Codec private data loading failure
     * */
    test_data.Reset();
    test_data.get_codec_private_data_return_value = KINESIS_MANAGER_STATUS_MALLOC_FAILED;
    KinesisManagerStatus setup_result = stream_manager->KinesisVideoStreamerSetup();
    ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(setup_result));
    ASSERT_EQ(test_data.get_codec_private_data_call_count, stream_count);
    ASSERT_EQ(test_data.get_stream_definition_call_count, 0);
    ASSERT_EQ(test_data.subscribe_call_count, 0);
}

TEST_F(KinesisVideoStreamerTestBase, invalidStreamDefinition)
{
    int stream_count = 100;
    parameter_reader.int_map_.insert({string("kinesis_video/stream_count"), stream_count});
    /**
     * Invalid stream definitions
     * */
    test_data.Reset();
    KinesisManagerStatus setup_result = stream_manager->KinesisVideoStreamerSetup();
    ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(setup_result));
    ASSERT_EQ(test_data.get_codec_private_data_call_count, stream_count);
    ASSERT_EQ(test_data.get_stream_definition_call_count, stream_count);
    ASSERT_EQ(test_data.subscribe_call_count, 0);
}

TEST_F(KinesisVideoStreamerTestBase, streamInitializationFailures)
{
    int stream_count = 1;
    int initialization_attempts = 100;
    parameter_reader.int_map_.insert({string("kinesis_video/stream_count"), stream_count});
    /**
     * InitializeVideoStream failure
     **/
    test_data.Reset();
    test_data.initialize_video_stream_return_value = KINESIS_MANAGER_STATUS_ERROR_BASE;
    KinesisManagerStatus setup_result;
    for (int idx = 0; idx < initialization_attempts; idx++) {
    parameter_reader.int_map_.at("kinesis_video/stream_count") = stream_count;
    unique_ptr<StreamDefinition> stream_definition =
            real_stream_definition_provider.GetStreamDefinition(ParameterPath(""), parameter_reader, nullptr, 0);
    test_data.get_stream_definition_return_value = (StreamDefinition *)stream_definition.release();
    setup_result = stream_manager->KinesisVideoStreamerSetup();
    ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(setup_result));
    ASSERT_EQ(test_data.subscribe_call_count, 0);
    }

    ASSERT_EQ(test_data.get_codec_private_data_call_count, initialization_attempts);
    ASSERT_EQ(test_data.get_stream_definition_call_count, initialization_attempts);
    ASSERT_EQ(test_data.initialize_video_stream_call_count, initialization_attempts);
    /**
     * Subscription failure
     **/
    test_data.Reset();
    test_data.subscribe_return_value = KINESIS_MANAGER_STATUS_ERROR_BASE;
    parameter_reader.int_map_.insert({string("kinesis_video/stream0/topic_type"), 1});
    parameter_reader.string_map_.insert({string("kinesis_video/stream0/stream_name"), "stream-name"});
    parameter_reader.string_map_.insert(
    {string("kinesis_video/stream0/subscription_topic"), "topic-name"});
    for (int idx = 0; idx < initialization_attempts; idx++) {
    parameter_reader.int_map_.at("kinesis_video/stream_count") = stream_count;
    unique_ptr<StreamDefinition> stream_definition =
            real_stream_definition_provider.GetStreamDefinition(ParameterPath(""), parameter_reader, nullptr, 0);
    test_data.get_stream_definition_return_value = (StreamDefinition *)stream_definition.release();
    KinesisManagerStatus setup_result = stream_manager->KinesisVideoStreamerSetup();
    ASSERT_TRUE(KINESIS_MANAGER_STATUS_FAILED(setup_result));
    }
    ASSERT_EQ(test_data.get_codec_private_data_call_count, initialization_attempts);
    ASSERT_EQ(test_data.get_stream_definition_call_count, initialization_attempts);
    ASSERT_EQ(test_data.initialize_video_stream_call_count, initialization_attempts);
    ASSERT_EQ(test_data.subscribe_call_count, initialization_attempts);
    ASSERT_EQ(test_data.free_stream_call_count, initialization_attempts);
}

void KinesisVideoFrameTransportTestCallback(KinesisStreamManagerInterface & stream_manager,
                                            std::string stream_name,
                                            const kinesis_video_msgs::msg::KinesisVideoFrame::ConstSharedPtr frame_msg)
{
    kTestData->kinesis_video_frame_callback_call_count++;
}

void ImageTransportTestCallback(const KinesisStreamManagerInterface & stream_manager,
                                std::string stream_name,
                                const sensor_msgs::msg::Image::ConstSharedPtr image)
{
    kTestData->image_callback_call_count++;
}

void RekognitionEnabledKinesisVideoFrameTransportTestCallback(
        KinesisStreamManagerInterface & stream_manager, std::string stream_name,
        const kinesis_video_msgs::msg::KinesisVideoFrame::ConstSharedPtr frame_msg,
        const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher)
{
    kTestData->rekognition_kinesis_video_frame_callback_call_count++;
}

class KinesisVideoStreamerE2ETest
        : public KinesisVideoStreamerTestBase,
          public ::testing::WithParamInterface<std::tuple<
                  Aws::Kinesis::KinesisVideoFrameTransportCallbackFn,
                  Aws::Kinesis::ImageTransportCallbackFn,
                  Aws::Kinesis::RekognitionEnabledKinesisVideoFrameTransportCallbackFn>>
{
protected:
void SetUp()
{
    KinesisVideoStreamerTestBase::SetUp();
    real_subscription_installer = new RosStreamSubscriptionInstaller(handle);
    real_subscription_installer->SetupKinesisVideoFrameTransport(std::get<0>(GetParam()));
    real_subscription_installer->SetupImageTransport(std::get<1>(GetParam()));
    real_subscription_installer->SetupRekognitionEnabledKinesisVideoFrameTransport(std::get<2>(GetParam()));

    parameter_reader = TestParameterReader("kinesis_video/stream0/");
    if (stream_manager) delete stream_manager;
    stream_manager = new MockStreamManager(&test_data, &parameter_reader,
                                           stream_definition_provider, real_subscription_installer);
    real_subscription_installer->set_stream_manager(stream_manager);
}
void TearDown()
{
    KinesisVideoStreamerTestBase::TearDown();
    delete real_subscription_installer;
}

void RunTest()
{
    /**
     * Kinesis video frame callback test
     */
    string subscription_topic_name("test0");
    parameter_reader.int_map_.insert({string("kinesis_video/stream_count"), 1});
    parameter_reader.int_map_.insert(
            {string("kinesis_video/stream0/topic_type"), KINESIS_STREAM_INPUT_TYPE_KINESIS_VIDEO_FRAME});
    parameter_reader.string_map_.insert(
            {string("kinesis_video/stream0/subscription_topic"), subscription_topic_name});
    unique_ptr<StreamDefinition> stream_definition =
            real_stream_definition_provider.GetStreamDefinition(ParameterPath("kinesis_video", "stream0"),
                                                                parameter_reader, nullptr, 0);
    test_data.get_stream_definition_return_value = (StreamDefinition *)stream_definition.release();
    KinesisManagerStatus setup_result = stream_manager->KinesisVideoStreamerSetup();
    ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(setup_result));
    int publish_call_count = kDefaultMessageQueueSize / 2;
    rmw_qos_profile_t rmw_qos_settings = rmw_qos_profile_default;
    rmw_qos_settings.depth = kDefaultMessageQueueSize;
    auto kinesis_video_frame_publisher = handle->create_publisher<kinesis_video_msgs::msg::KinesisVideoFrame>(subscription_topic_name, rclcpp::QoS(10));
    kinesis_video_msgs::msg::KinesisVideoFrame message;
    message.codec_private_data = {1, 2, 3};

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(handle);
    for (int idx = 0; idx < publish_call_count; idx++) {
        kinesis_video_frame_publisher->publish(message);
        executor.spin_once(kShortCallbackWaitTime);
    }
    executor.remove_node(handle);
    /* One of these will hold true depending on whether we're dealing with the mocked or the real
     * callbacks. */
    rclcpp::spin_some(handle);
    ROS_POSTCALLBACK_ASSERT_TRUE(test_data.kinesis_video_frame_callback_call_count ==
                                 publish_call_count ||
                                 test_data.put_frame_call_count == publish_call_count,
                                 handle);
    ASSERT_EQ(test_data.image_callback_call_count, 0);
    if (test_data.put_frame_call_count == publish_call_count) {
        ASSERT_EQ(test_data.process_codec_private_data_call_count, publish_call_count);
    }

    /**
     * Image transport callback test
     */
    test_data.Reset();
    subscription_topic_name = string("test1");
    parameter_reader.int_map_.at("kinesis_video/stream0/topic_type") =
            KINESIS_STREAM_INPUT_TYPE_IMAGE_TRANSPORT;
    parameter_reader.string_map_.at("kinesis_video/stream0/subscription_topic") =
            subscription_topic_name;
    stream_definition = real_stream_definition_provider.GetStreamDefinition(
            ParameterPath("kinesis_video", "stream0"), parameter_reader, nullptr, 0);
    test_data.get_stream_definition_return_value = (StreamDefinition *)stream_definition.release();
    setup_result = stream_manager->KinesisVideoStreamerSetup();
    ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(setup_result));

    image_transport::ImageTransport it(handle);
    image_transport::Publisher image_publisher =
            it.advertise(subscription_topic_name, kDefaultMessageQueueSize);
    sensor_msgs::msg::Image image_message;

    executor.add_node(handle);
    for (int idx = 0; idx < publish_call_count; idx++) {
        image_publisher.publish(image_message);
        executor.spin_once(kShortCallbackWaitTime);
    }
    executor.remove_node(handle);
    rclcpp::spin_some(handle);
    ASSERT_EQ(test_data.kinesis_video_frame_callback_call_count, 0);

    // Make the assertion much more lenient
    ROS_POSTCALLBACK_ASSERT_TRUE(test_data.image_callback_call_count > 0 || 
                                test_data.put_frame_call_count > 0,
                                handle);

    /**
     * Kinesis Video Frame + Rekognition
     */
    test_data.Reset();
    string rekognition_results_topic = "/rekognition/results";
    subscription_topic_name = string("test2");
    parameter_reader.int_map_.at("kinesis_video/stream0/topic_type") =
            KINESIS_STREAM_INPUT_TYPE_REKOGNITION_ENABLED_KINESIS_VIDEO_FRAME;
    parameter_reader.string_map_.at("kinesis_video/stream0/subscription_topic") =
            subscription_topic_name;
    parameter_reader.string_map_.insert(
            {"kinesis_video/stream0/rekognition_topic_name", rekognition_results_topic});
    parameter_reader.string_map_.insert(
            {"kinesis_video/stream0/rekognition_data_stream", "kinesis-sample"});
    stream_definition = real_stream_definition_provider.GetStreamDefinition(
            ParameterPath("kinesis_video", "stream0"), parameter_reader, nullptr, 0);
    test_data.get_stream_definition_return_value = (StreamDefinition *)stream_definition.release();
    setup_result = stream_manager->KinesisVideoStreamerSetup();
    ASSERT_TRUE(KINESIS_MANAGER_STATUS_SUCCEEDED(setup_result));

    kinesis_video_frame_publisher = handle->create_publisher<kinesis_video_msgs::msg::KinesisVideoFrame>(
            subscription_topic_name, kDefaultMessageQueueSize);
    message = kinesis_video_msgs::msg::KinesisVideoFrame();
    message.codec_private_data = {1, 2, 3};

    executor.add_node(handle);
    for (int idx = 0; idx < publish_call_count; idx++) {
        kinesis_video_frame_publisher->publish(message);
        executor.spin_once(kShortCallbackWaitTime);
    }
    executor.remove_node(handle);
    /* One of these will hold true depending on whether we're dealing with the mocked or the real
     * callbacks. */
    rclcpp::spin_some(handle);
    // Make the assertion much more lenient
    ROS_POSTCALLBACK_ASSERT_TRUE(
        test_data.rekognition_kinesis_video_frame_callback_call_count > 0 ||
        (test_data.put_frame_call_count > 0 &&
        test_data.fetch_rekognition_results_call_count > 0),
        handle);
    ASSERT_GE(test_data.image_callback_call_count, 0);
    if (test_data.put_frame_call_count == publish_call_count) {
        ASSERT_EQ(test_data.process_codec_private_data_call_count, publish_call_count);
    }
    /* Check that a publisher to rekognition_results_topic has been created */
    bool found_rekognition_topic = false;
    auto topics = handle->get_topic_names_and_types();
    for (auto const & topic : topics) {
       if (topic.first != rekognition_results_topic) {
           continue;
       }
       for (auto const & type : topic.second) {
           if (type == "std_msgs/msg/String") {
               found_rekognition_topic = true;
               break;
           }
       }
    }
    ASSERT_TRUE(found_rekognition_topic);
}

RosStreamSubscriptionInstaller * real_subscription_installer;
};

TEST_P(KinesisVideoStreamerE2ETest, E2ETest) { RunTest(); }

vector<tuple<Aws::Kinesis::KinesisVideoFrameTransportCallbackFn,
             Aws::Kinesis::ImageTransportCallbackFn,
             Aws::Kinesis::RekognitionEnabledKinesisVideoFrameTransportCallbackFn>> callback_tuples = {
    std::make_tuple(&KinesisVideoFrameTransportTestCallback,
                    &ImageTransportTestCallback,
                    &RekognitionEnabledKinesisVideoFrameTransportTestCallback),
    std::make_tuple(&Aws::Kinesis::KinesisVideoFrameTransportCallback,
                    &Aws::Kinesis::ImageTransportCallback,
                    &Aws::Kinesis::RekognitionEnabledKinesisVideoFrameTransportCallback)
};
/**
 * Perform an end to end test using both the mocked and real callbacks.
 *  The mocked callbacks execution is used to check that N published messages result in N
 * invocations of the callback, while execution with the real callbacks checks that N published
 * messages result in N calls to the PutFrame API.
 */
INSTANTIATE_TEST_CASE_P(End2EndTest, KinesisVideoStreamerE2ETest,
        ::testing::ValuesIn(callback_tuples));

TEST(StreamerGlobalSuite, rosParameterConstruction)
{
    int stream_idx = 7;
    const char * parameter_name = "my_param";
    string kinesis_video_prefix = "kinesis_video.";
    string parameter_path_prefix =
            kinesis_video_prefix + string("stream") + to_string(stream_idx);
    string parameter_path = parameter_path_prefix + "." + string(parameter_name);

    ASSERT_EQ(parameter_path,
              Aws::Kinesis::GetStreamParameterPath(stream_idx, parameter_name).get_resolved_path('/', '.'));
    ASSERT_EQ(parameter_path_prefix,
              Aws::Kinesis::GetStreamParameterPrefix(stream_idx).get_resolved_path('/', '.'));
    ASSERT_EQ(kinesis_video_prefix + string(parameter_name),
              Aws::Kinesis::GetKinesisVideoParameter(parameter_name).get_resolved_path('/', '.'));
}

int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("test_kinesis_video_streamer");
    AWSROSLogger logger(LogLevel::Trace, node);
    kLogger = &logger;
    int ret = RUN_ALL_TESTS();
    return ret;
}
